local M = {}

local generation = 0
local timer = 0

M.raw_transforms = {}
M.received_transforms = {}
M.local_transforms = {}
M.raw_positions = {}
M.inactive = {}

M.threshold = 3
M.rot_threshold = 2.5
M.velocity_error_limit = 10

M.hidden = {}

-- BeamNG auto-loads lua/vehicle/extensions/*.lua but does not recurse into
-- subfolders. The kiss_mp/* extensions need an explicit addModulePath +
-- loadModulesInDirectory call to become available in vehicle Lua. Prepended
-- to every queueLuaCommand into a kiss_mp/* module so the call is self-healing
-- if the vehicle Lua context ever resets.
local VEHICLE_SYNC_BOOTSTRAP = "extensions.addModulePath('lua/vehicle/extensions/kiss_mp'); extensions.loadModulesInDirectory('lua/vehicle/extensions/kiss_mp'); "

local function queue_kiss_command(vehicle, command)
  if not vehicle then return end
  vehicle:queueLuaCommand(VEHICLE_SYNC_BOOTSTRAP .. command)
end

-- Direct pose snap, used only for large recovery corrections. The target
-- position passed here must be a refnode/origin position, not COG.
local function apply_motion_target(vehicle_id,
                                   target_origin_x, target_origin_y, target_origin_z,
                                   target_rotation_x, target_rotation_y, target_rotation_z, target_rotation_w,
                                   target_velocity_x, target_velocity_y, target_velocity_z)
  local veh = be:getObjectByID(vehicle_id)
  if not veh then return end
  local ref_node_id = veh:getRefNodeId()

  local current_rotation = quatFromDir(-veh:getDirectionVector(), veh:getDirectionVectorUp())
  local target_rotation = quat(target_rotation_x, target_rotation_y, target_rotation_z, target_rotation_w)
  local relative_rotation = current_rotation:inversed() * target_rotation

  veh:setClusterPosRelRot(ref_node_id, target_origin_x, target_origin_y, target_origin_z,
    relative_rotation.x, relative_rotation.y, relative_rotation.z, relative_rotation.w)

  -- The pose snap rotates the body without touching its velocity, so the
  -- existing velocity is still expressed in the old orientation. Rotate it to
  -- match before correcting it to the target.
  local local_velocity = vec3(veh:getVelocity())
  local rotated_local_velocity = local_velocity:rotated(relative_rotation)
  veh:applyClusterVelocityScaleAdd(ref_node_id, 1,
    target_velocity_x - rotated_local_velocity.x,
    target_velocity_y - rotated_local_velocity.y,
    target_velocity_z - rotated_local_velocity.z)
end

local function queue_cog_snap(vehicle, transform)
  local position, rotation = transform.position, transform.rotation
  local velocity = transform.velocity or {0, 0, 0}
  local angular_velocity = transform.angular_velocity or {0, 0, 0}
  if not (position and rotation and #position >= 3 and #rotation >= 4) then return end
  queue_kiss_command(vehicle,
    "kiss_motion_controller.snap_to_cog_target("
    ..position[1]..","..position[2]..","..position[3]..","
    ..rotation[1]..","..rotation[2]..","..rotation[3]..","..rotation[4]..","
    ..(velocity[1] or 0)..","..(velocity[2] or 0)..","..(velocity[3] or 0)..","
    ..(angular_velocity[1] or 0)..","..(angular_velocity[2] or 0)..","..(angular_velocity[3] or 0)..")"
  )
end

local function update(dt)
  if not network.connection.connected then return end

  -- Refresh each vehicle's local transform cache. Only owned vehicles send
  -- this cache over the network, but remote vehicles still need their vehicle
  -- Lua modules loaded before receiver-side correction runs.
  for i = 0, be:getObjectCount() do
    local vehicle = be:getObject(i)
    local vid = vehicle and vehicle:getID()
    if vehicle and (not M.inactive[vid]) then
      local owned = vehiclemanager.ownership[vid] ~= nil
      queue_kiss_command(vehicle, "kiss_vehicle.update_transform_info(" .. tostring(owned) .. ")")
    end
  end

  -- Don't apply velocity while paused. If we do, velocity gets stored up and released when the game resumes.
  local apply_velocity = not bullettime.getPause()
  for id, transform in pairs(M.received_transforms) do
    --apply_transform(dt, id, transform, apply_velocity)
    local vehicle = be:getObjectByID(id)
    local p = vec3(transform.position)
    if vehicle and apply_velocity and (not vehiclemanager.ownership[id]) then
      if ((p:distance(vec3(getCameraPosition())) > kissui.view_distance[0])) and kissui.enable_view_distance[0] then
        if (not M.inactive[id]) then
          vehicle:setActive(0)
          M.inactive[id] = true
        end
      else
        if M.inactive[id] then
          vehicle:setActive(1)
          M.inactive[id] = false
          -- Reactivated replicas can be far from the authority because
          -- setActive(0) freezes local physics. Snap once, then resume the
          -- normal per-frame correction path.
          queue_cog_snap(vehicle, transform)
        end
        -- Per-frame correction runs from kiss_motion_controller.updateGFX
        -- inside vehicle Lua. GE only handles activity/view-distance here.
      end
    end
  end
end

local function update_vehicle_transform(data)
  local transform = data.transform
  transform.owner = data.vehicle_id
  transform.sent_at = data.sent_at
  transform.send_timer = data.send_timer
  transform.ping_ms = data.ping_ms
  -- Our own latency to the server. Together with the sender's ping_ms this
  -- covers both legs of the path the packet actually travelled.
  transform.receiver_ping_ms = network.connection.rtt_smooth_ms or network.connection.ping or 0

  local id = vehiclemanager.id_map[transform.owner or -1] or -1
  if vehiclemanager.ownership[id] then return end
  M.raw_positions[transform.owner or -1] = transform.position
  M.received_transforms[id] = transform

  local vehicle = be:getObjectByID(id)
  if vehicle and (not M.inactive[id]) then
    -- Packet arrival hands the new authoritative COG pose to kiss_sync.
    -- Application happens per-frame from kiss_motion_controller.update(dt),
    -- not on packet arrival.
    queue_kiss_command(vehicle, "kiss_motion_controller.set_target_transform(" .. string.format("%q", jsonEncode(transform)) .. ")")
  end
end

local function push_transform(id, t)
  M.local_transforms[id] = jsonDecode(t)
end

M.send_transform_updates = send_transform_updates
M.send_vehicle_transform = send_vehicle_transform
M.update_vehicle_transform = update_vehicle_transform
M.push_transform = push_transform
M.queue_kiss_command = queue_kiss_command
M.queue_cog_snap = queue_cog_snap
M.apply_motion_target = apply_motion_target
M.onUpdate = update

return M
