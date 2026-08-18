local M = {}
local parts_config = v.config

local nodes = {}
local node_by_cid = {}
local connected_node_states = {}
local connected_node_set = {}
local connected_graph = {}
local parent_node = nil
local last_damage = 0

-- Superseded force-based replay state, still driven by kiss_transforms.lua.
local legacy_nodes = {}
local ref_nodes = {}
local last_node = 1
local nodes_per_frame = 32
local node_pos_thresh = 3
local node_pos_thresh_sqr = node_pos_thresh * node_pos_thresh

M.test_quat = quat(0.707, 0, 0, 0.707)

-- Mass-weighted centre of gravity in body frame. Published pose and twist are
-- anchored here rather than at the refnode: the refnode sits wherever the jbeam
-- author put it, so during a turn its motion mixes in a lever arm the receiver
-- cannot reconstruct, and receiver-side correction then chases that phantom
-- error. The COG is the one point both sides can agree on.
M.sync_cog_body = vec3(0, 0, 0)

local last_cog_compute_time = -math.huge
-- Seconds between COG recomputations. The COG only moves as mass moves - i.e.
-- with damage and fuel burn - so it is slow relative to physics, and walking
-- the whole node table every step would be thousands of traversals a second
-- for a value that barely changes. Damage forces an immediate recompute.
local COG_RECOMPUTE_INTERVAL_S = 0.2

-- Cutoff in Hz for the lowpass on published velocity and angular velocity.
-- Raw per-step soft-body velocity carries solver ringing the receiver cannot
-- tell apart from real motion; it gets differentiated into acceleration there
-- and amplified. 50 Hz sits above the body motion worth replicating and below
-- the node-level chatter. Value from live tuning.
local SEND_SMOOTH_RATE = 50.0
local smoothed_send_refnode_velocity = nil
local smoothed_send_body_angular_velocity = nil
local last_motion_sample_time = nil
local last_motion_sample_dt = 1/60
local cached_transform_sample = nil
local send_timer = 0

local function get_body_gyro_local_omega()
  return vec3(
    obj:getPitchAngularVelocity(),
    obj:getRollAngularVelocity(),
    obj:getYawAngularVelocity()
  )
end

local function lowpass_dt(prev, target, dt, rate)
  if not prev then
    return vec3(target.x, target.y, target.z)
  end
  local alpha = math.min(rate * dt, 1.0)
  return vec3(
    prev.x + (target.x - prev.x) * alpha,
    prev.y + (target.y - prev.y) * alpha,
    prev.z + (target.z - prev.z) * alpha
  )
end

local function reset_send_smoothers()
  smoothed_send_refnode_velocity = nil
  smoothed_send_body_angular_velocity = nil
  last_motion_sample_time = nil
  last_motion_sample_dt = 1/60
  cached_transform_sample = nil
end

local function get_refnode_ids()
  local refs = v and v.data and v.data.refNodes
  if type(refs) ~= "table" then return {} end
  local entry = refs[0] or refs[1] or refs
  return {
    entry and (entry.ref or entry.idRef or entry.cidRef),
    entry and (entry.back or entry.idX or entry.cidX),
    entry and (entry.up or entry.idY or entry.cidY),
    entry and (entry.left or entry.idLeft or entry.cidLeft),
  }
end

local function resolve_cid(value)
  if type(value) == "number" and node_by_cid[value] then return value end
  if type(value) == "string" then
    local numeric = tonumber(value)
    if numeric and node_by_cid[numeric] then return numeric end
    if beamstate and beamstate.nodeNameMap then
      local mapped = beamstate.nodeNameMap[value]
      if mapped and node_by_cid[mapped] then return mapped end
    end
  end
  return nil
end

-- Beam adjacency over structural beams only. Types 3, 4 and 7 are support,
-- pressure and bounded beams, which link parts that are not rigidly attached
-- and would merge separate bodies into a single graph.
local function build_connected_graph()
  connected_graph = {}
  if not (v and v.data and v.data.beams) then return end

  for _, beam in pairs(v.data.beams) do
    if beam.beamType ~= 3 and beam.beamType ~= 4 and beam.beamType ~= 7 then
      local a = resolve_cid(beam.id1)
      local b = resolve_cid(beam.id2)
      if a and b then
        connected_graph[a] = connected_graph[a] or {}
        connected_graph[b] = connected_graph[b] or {}
        connected_graph[a][#connected_graph[a] + 1] = {cid = b, beam = beam.cid}
        connected_graph[b][#connected_graph[b] + 1] = {cid = a, beam = beam.cid}
      end
    end
  end
end

local function choose_parent_node()
  parent_node = nil
  for _, ref in ipairs(get_refnode_ids()) do
    local cid = resolve_cid(ref)
    if cid and connected_graph[cid] then
      parent_node = cid
      return
    end
  end
  for cid in pairs(node_by_cid) do
    parent_node = cid
    return
  end
end

-- Flood-fill from the refnode across unbroken beams. Nodes that broke away are
-- no longer part of the body being replicated, so they must not drag the COG
-- toward wherever the debris ended up.
local function rebuild_connected_nodes()
  connected_node_states = {}
  connected_node_set = {}
  if not parent_node then
    for _, state in ipairs(nodes) do
      connected_node_states[#connected_node_states + 1] = state
      connected_node_set[state.cid] = true
    end
    return
  end

  local stack = {parent_node}
  connected_node_set[parent_node] = true
  while #stack > 0 do
    local cid = stack[#stack]
    stack[#stack] = nil
    local state = node_by_cid[cid]
    if state then
      connected_node_states[#connected_node_states + 1] = state
    end
    for _, edge in ipairs(connected_graph[cid] or {}) do
      local other = edge.cid
      if not connected_node_set[other] then
        local broken = edge.beam ~= nil and obj:beamIsBroken(edge.beam)
        if not broken then
          connected_node_set[other] = true
          stack[#stack + 1] = other
        end
      end
    end
  end
end

-- Mass-weighted COG offset in body frame. The receiver uses this same body
-- offset to run its correction loop in COG-space instead of refnode-space.
local function compute_sync_cog_body()
  local total_mass = 0
  local cog_sum_x, cog_sum_y, cog_sum_z = 0, 0, 0

  local cog_nodes = (#connected_node_states > 0) and connected_node_states or nodes
  for _, state in ipairs(cog_nodes) do
    local mass = state.mass or 0
    if mass > 0 then
      local pos = obj:getNodePosition(state.cid)
      if pos then
        cog_sum_x = cog_sum_x + pos.x * mass
        cog_sum_y = cog_sum_y + pos.y * mass
        cog_sum_z = cog_sum_z + pos.z * mass
        total_mass = total_mass + mass
      end
    end
  end

  if total_mass < 1e-9 then
    M.sync_cog_body = vec3(0, 0, 0)
    return
  end

  local inv = 1 / total_mass
  local cog_offset_world = vec3(cog_sum_x * inv, cog_sum_y * inv, cog_sum_z * inv)
  local rot = quatFromDir(-vec3(obj:getDirectionVector()), vec3(obj:getDirectionVectorUp()))
  M.sync_cog_body = cog_offset_world:rotated(rot:inversed())
end

local function maybe_recompute_sync_cog_body()
  local now = os.clock()
  local damage = (beamstate and beamstate.damage) or 0
  if damage ~= last_damage then
    rebuild_connected_nodes()
    last_damage = damage
    last_cog_compute_time = -math.huge
  end
  if now - last_cog_compute_time >= COG_RECOMPUTE_INTERVAL_S then
    compute_sync_cog_body()
    last_cog_compute_time = now
  end
end

local function get_sync_cog_body()
  return M.sync_cog_body or vec3(0, 0, 0)
end

local function get_disconnected_node_states()
  local out = {}
  for _, state in ipairs(nodes) do
    if not connected_node_set[state.cid] then
      out[#out + 1] = state
    end
  end
  return out
end

local function get_smoothed_local_motion()
  return {
    refnode_velocity = smoothed_send_refnode_velocity,
    body_omega = smoothed_send_body_angular_velocity,
    dt = last_motion_sample_dt,
  }
end

local function update_motion_sample(dt)
  -- Sampling runs on the physics step, not the graphics frame: velocity read on
  -- a graphics frame has already been resampled at a rate unrelated to the
  -- solver. The send path only packages what was captured here.
  local now = obj:getSimTime() or os.clock()
  if last_motion_sample_time ~= nil and now <= last_motion_sample_time then
    return
  end

  local sample_dt = dt
  if not sample_dt or sample_dt <= 0 then
    sample_dt = last_motion_sample_time and (now - last_motion_sample_time) or (1/60)
  end
  if sample_dt <= 0 then sample_dt = 1/60 end
  if sample_dt > 0.1 then sample_dt = 0.1 end

  local r = quatFromDir(-vec3(obj:getDirectionVector()), vec3(obj:getDirectionVectorUp()))
  local p = vec3(obj:getPosition())
  local raw_refnode_velocity = vec3(obj:getVelocity())
  local raw_body_angular_velocity = get_body_gyro_local_omega()

  smoothed_send_refnode_velocity = lowpass_dt(smoothed_send_refnode_velocity, raw_refnode_velocity, sample_dt, SEND_SMOOTH_RATE)
  smoothed_send_body_angular_velocity = lowpass_dt(smoothed_send_body_angular_velocity, raw_body_angular_velocity, sample_dt, SEND_SMOOTH_RATE)

  local refnode_velocity_world = smoothed_send_refnode_velocity
  local angular_velocity_world = smoothed_send_body_angular_velocity:rotated(r)

  maybe_recompute_sync_cog_body()
  local cog_offset_world = get_sync_cog_body():rotated(r)
  local cog_position_world = p + cog_offset_world
  -- Rigid-body transfer of the twist from refnode to COG: v_cog = v_ref + r x w.
  local cog_velocity_world = refnode_velocity_world + cog_offset_world:cross(angular_velocity_world)

  last_motion_sample_time = now
  last_motion_sample_dt = sample_dt
  send_timer = now
  cached_transform_sample = {
    position = cog_position_world,
    rotation = r,
    velocity = cog_velocity_world,
    angular_velocity = angular_velocity_world,
  }
end

local function onExtensionLoaded()
  nodes = {}
  node_by_cid = {}
  connected_node_states = {}
  connected_node_set = {}
  parent_node = nil
  last_damage = (beamstate and beamstate.damage) or 0
  last_cog_compute_time = -math.huge
  reset_send_smoothers()
  send_timer = 0

  if v and v.data and v.data.nodes then
    for _, node in pairs(v.data.nodes) do
      if node.cid ~= nil then
        local state = {
          cid = node.cid,
          mass = obj:getNodeMass(node.cid),
        }
        nodes[#nodes + 1] = state
        node_by_cid[node.cid] = state
      end
    end
  end

  build_connected_graph()
  choose_parent_node()
  rebuild_connected_nodes()
  compute_sync_cog_body()

  -- Superseded force-based replay state.
  local force = obj:getPhysicsFPS()
  local ref = {
    v.data.refNodes[0].left,
    v.data.refNodes[0].up,
    v.data.refNodes[0].back,
    v.data.refNodes[0].ref,
  }
  local total_mass = 0
  local inverse_rot = quat(obj:getRotation()):inversed()
  for _, node in pairs(v.data.nodes) do
    local node_mass = obj:getNodeMass(node.cid)
    local node_pos = inverse_rot * obj:getNodePosition(node.cid)
    table.insert(legacy_nodes, {node.cid, node_mass * force, true, node_pos})
    total_mass = total_mass + node_mass
  end
  for _, node in pairs(ref) do
    table.insert(
      ref_nodes,
      {node, total_mass * force / 4, true, inverse_rot * obj:getNodePosition(node)}
    )
  end
end

local function onReset()
  last_cog_compute_time = -math.huge
  last_damage = (beamstate and beamstate.damage) or 0
  rebuild_connected_nodes()
  reset_send_smoothers()
  compute_sync_cog_body()
end

-- A local teleport moves the body with no physics continuity, so everything
-- derived from the previous position is meaningless afterwards.
local function post_owner_teleport_settle()
  reset_send_smoothers()
  last_cog_compute_time = -math.huge
  compute_sync_cog_body()
end

  -- NOTE:
  -- This is a temperary solution. It's not great. We made it to release the mod.
  -- A better solution will be used in future versions
local function update_eligible_nodes()
  local inverse_rot =  quat(obj:getRotation()):inversed()
  for k=last_node, math.min(#legacy_nodes , last_node + nodes_per_frame) do
    local node = legacy_nodes[k]
    local local_node_pos = inverse_rot * obj:getNodePosition(node[1])
    local local_original_pos = node[4]
    node[3] = (local_node_pos - local_original_pos):squaredLength() < node_pos_thresh_sqr
    last_node = k
  end
  if last_node == #legacy_nodes then last_node = 1 end
end

local function update_transform_info(_we_own_this_vehicle)
  update_motion_sample()
  local sample = cached_transform_sample
  if not sample then return end

  local throttle_input = electrics.values.throttle_input or 0
  local brake_input = electrics.values.brake_input or 0
  if electrics.values.gearboxMode == "arcade" and electrics.values.gearIndex < 0 then
    throttle_input, brake_input = brake_input, throttle_input
  end

  local input = {
    vehicle_id = obj:getID() or 0,
    throttle_input = throttle_input,
    brake_input = brake_input,
    clutch = electrics.values.clutch_input or 0,
    parkingbrake = electrics.values.parkingbrake_input or 0,
    steering_input = electrics.values.steering_input or 0,
  }
  local transform = {
    position = {sample.position.x, sample.position.y, sample.position.z},
    rotation = {sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w},
    velocity = {sample.velocity.x, sample.velocity.y, sample.velocity.z},
    angular_velocity = {sample.angular_velocity.x, sample.angular_velocity.y, sample.angular_velocity.z},
    input = input,
    gearbox = kiss_gearbox.get_gearbox_data(),
    send_timer = send_timer,
    -- Age of the sample when it was captured, so the receiver can account for
    -- the part of the delay that happened before the packet was even built.
    send_dt = last_motion_sample_dt,
  }
  obj:queueGameEngineLua("kisstransform.push_transform("..obj:getID()..", " .. string.format("%q", jsonEncode(transform)) .. ")")
end

local function apply_linear_velocity(x, y, z)
  local velocity = vec3(x, y, z)
  local force = float3(0, 0, 0)
  for k=1, #legacy_nodes do
    local node = legacy_nodes[k]
    if node[3] then
      local result = velocity * node[2]
      force:set(result.x, result.y, result.z)
      obj:applyForceVector(node[1], force)
    end
  end
end

local function apply_linear_velocity_ang_torque(x, y, z, pitch, roll, yaw)
  local velocity = vec3(x, y, z)
  local nodes = legacy_nodes
  local rot = vec3(pitch, roll, yaw):rotated(quat(obj:getRotation()))
  local node_position = vec3()
  local force = float3(0, 0, 0)
  for k=1, #nodes do
    local node = nodes[k]
    if node[3] then
      node_position:set(obj:getNodePosition(node[1]))
      local result = (velocity + node_position:cross(rot)) * node[2]
      force:set(result.x, result.y, result.z)
      obj:applyForceVector(node[1], force)
    end
  end
end

local function send_vehicle_config()
  local r = quatFromDir(-vec3(obj:getDirectionVector()), vec3(obj:getDirectionVectorUp()))
  local p = obj:getPosition()
  local data = {
    position = {p.x, p.y, p.z},
    rotation = {r.x, r.y, r.z, r.w},
  }
  obj:queueGameEngineLua("vehiclemanager.send_vehicle_config_inner("..obj:getID()..", " .. string.format("%q", jsonEncode(v.config)) .. ", " .. string.format("%q", jsonEncode(data)) .. ")")
end

M.update_transform_info = update_transform_info
M.onPhysicsStep = update_motion_sample
M.get_sync_cog_body = get_sync_cog_body
M.get_disconnected_node_states = get_disconnected_node_states
M.get_smoothed_local_motion = get_smoothed_local_motion
M.maybe_recompute_sync_cog_body = maybe_recompute_sync_cog_body
M.compute_sync_cog_body = compute_sync_cog_body
M.onExtensionLoaded = onExtensionLoaded
M.onReset = onReset
M.post_owner_teleport_settle = post_owner_teleport_settle
M.send_vehicle_config = send_vehicle_config

M.apply_linear_velocity_ang_torque = apply_linear_velocity_ang_torque
M.update_eligible_nodes = update_eligible_nodes
M.apply_linear_velocity = apply_linear_velocity

return M
