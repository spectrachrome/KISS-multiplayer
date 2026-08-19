local M = {}
local imgui = ui_imgui

-- A/B TEST SCAFFOLDING (not for upstream).
--
-- Created lazily inside draw(): this file is require()d from kissui.lua at its
-- file scope, which runs before the ui_imgui dependency is resolved. Calling
-- into imgui at module scope indexes a nil and takes kissui down with it.
local legacy_sync = nil

local function draw()
  legacy_sync = legacy_sync or imgui.BoolPtr(false)

  imgui.Separator()
  imgui.Text("Sync path (testing)")
  if imgui.Checkbox("Use legacy force-based sync", legacy_sync) then
    kisstransform.set_legacy_sync(legacy_sync[0])
  end
  imgui.PushTextWrapPos(0)
  imgui.Text("Off = COG motion sync with dead-reckoning and PD correction. On = the old per-node force replay. Both clients must be set the same way, or the two ends will disagree about what the packet's position means.")
  imgui.PopTextWrapPos()
  imgui.Separator()

  if imgui.Checkbox("Show Name Tags", kissui.show_nametags) then
    kissconfig.save_config()
  end
  if imgui.Checkbox("Show Players In Vehicles", kissui.show_drivers) then
    kissconfig.save_config()
  end
  imgui.Text("Window Opacity")
  imgui.SameLine()
  if imgui.SliderFloat("###window_opacity", kissui.window_opacity, 0, 1) then
    kissconfig.save_config()
  end
  if imgui.Checkbox("Enable view distance (Experimental)", kissui.enable_view_distance) then
    kissconfig.save_config()
  end
  if kissui.enable_view_distance[0] then
    if imgui.SliderInt("###view_distance", kissui.view_distance, 50, 1000) then
      kissconfig.save_config()
    end
    imgui.PushTextWrapPos(0)
    imgui.Text("Warning. This feature is experimental. It can introduce a small, usually unnoticeable lag spike when approaching nearby vehicles. It'll also block the ability to switch to far away vehicles")
    imgui.PopTextWrapPos()
  end
end

M.draw = draw

return M
