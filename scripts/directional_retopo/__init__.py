"""Maya-side helpers for the DirectionalRetopo plug-in."""

from importlib import reload as _reload

from . import runtime, tool


# Maya development sessions commonly reload the package without restarting.
# A module reload preserves this sentinel in the package dictionary, allowing
# the second and subsequent package executions to refresh the helper submodule.
if globals().get("_DIRECTIONAL_RETOPO_IMPORTED", False):
    _reload(tool)
_DIRECTIONAL_RETOPO_IMPORTED = True

activate = tool.activate
deactivate = tool.deactivate
load_plugin = tool.load_plugin
query_radius = tool.query_radius
query_radius_adjust_mode = tool.query_radius_adjust_mode
query_density_mode = tool.query_density_mode
query_edge_length_scale = tool.query_edge_length_scale
query_field_visualization = tool.query_field_visualization
query_manual_target_edge_length = tool.query_manual_target_edge_length
query_quad_preview_visibility = tool.query_quad_preview_visibility
query_quad_preview_layers = tool.query_quad_preview_layers
query_boundary_preview_layers = tool.query_boundary_preview_layers
query_boundary_debug_visualization = tool.query_boundary_debug_visualization
query_topology_blend_width = tool.query_topology_blend_width
set_radius = tool.set_radius
set_radius_adjust_mode = tool.set_radius_adjust_mode
set_density_mode = tool.set_density_mode
set_edge_length_scale = tool.set_edge_length_scale
set_field_visualization = tool.set_field_visualization
set_manual_target_edge_length = tool.set_manual_target_edge_length
set_quad_preview_visibility = tool.set_quad_preview_visibility
set_quad_preview_layers = tool.set_quad_preview_layers
set_boundary_debug_visualization = tool.set_boundary_debug_visualization
set_topology_blend_width = tool.set_topology_blend_width
reset_settings = tool.reset_settings
unload_plugin = tool.unload_plugin

__all__ = [
    "activate",
    "deactivate",
    "load_plugin",
    "query_radius_adjust_mode",
    "query_density_mode",
    "query_edge_length_scale",
    "query_field_visualization",
    "query_manual_target_edge_length",
    "query_quad_preview_visibility",
    "query_quad_preview_layers",
    "query_boundary_preview_layers",
    "query_boundary_debug_visualization",
    "query_topology_blend_width",
    "query_radius",
    "runtime",
    "set_radius_adjust_mode",
    "set_radius",
    "set_density_mode",
    "set_edge_length_scale",
    "set_field_visualization",
    "set_manual_target_edge_length",
    "set_quad_preview_visibility",
    "set_quad_preview_layers",
    "set_boundary_debug_visualization",
    "set_topology_blend_width",
    "reset_settings",
    "tool",
    "unload_plugin",
]
