# DirectionalRetopo

DirectionalRetopo is a Maya 2024.2 C++ tool plug-in for collecting directional brush strokes and previewing a read-only, boundary-locked, quad-dominant local remesh on an existing polygon mesh. Phase 5A.1 keeps the Source Boundary fixed, joins it to an AutoRemesher-generated inner patch through a Quad/Triangle Transition Collar, validates curved boundaries in 3D, and displays the result through persistent Viewport 2.0 drawables without modifying source geometry or topology.

## Fixed build target

- Windows x64
- Autodesk Maya 2024.2 / API `20240200`
- Maya 2024 Update 2 DevKit
- Visual Studio Community 2022
- MSVC v143 x64
- CMake generator `Visual Studio 17 2022`
- Release configuration

Build from a normal PowerShell or Command Prompt. A Developer Command Prompt is not required.

```bat
build_maya2024.bat
```

The script uses the standard Maya 2024 install location and looks for the Maya
2024.2 DevKit at `..\devkit\maya2024.2\devkitBase` relative to this repository.
Set `MAYA_LOCATION` or `MAYA_DEVKIT_ROOT` before running it when your local
layout differs; Maya and DevKit files remain external to the repository.

The resulting plug-in is written to:

```text
build\Release\DirectionalRetopo.mll
```

Unload `DirectionalRetopo.mll` from every running Maya process before rebuilding. Windows locks a loaded plug-in binary and the linker will otherwise report `LNK1104`.

If the Python helper is already imported in Maya, this safely switches away from the tool, deletes its Context instance, and unloads the plug-in:

```python
from directional_retopo import tool
tool.unload_plugin()
```

## Phase 1.8 foundation through Phase 4

- Registers the `directionalRetopoContext` context command and the future undo integration command `directionalRetopoToolCmd`.
- Uses one selected, non-intermediate polygon mesh as the paint target.
- Converts viewport mouse coordinates to a world-space ray with `M3dView::viewToWorld()`.
- Finds the closest surface hit with `MFnMesh::closestIntersection()` and Maya's cached automatic uniform-grid intersection accelerator.
- Records world position, surface normal, face/triangle identifiers, barycentric values, brush radius, weight, and surface-tangent stroke direction in `StrokeSample` values.
- Samples at a radius-relative spacing rather than on every mouse event.
- Draws Region Preview, Stroke, and Direction feedback through the Context while keeping Brush Circle and Target Wireframe submission completely outside `MPxContext::drawFeedback()`.
- Registers one temporary `directionalRetopoBrushCursorShape` (`MPxLocatorNode`) and one `DirectionalRetopoBrushCursorDrawOverride` (`MPxDrawOverride`) while the tool is active. The DrawOverride owns the only Brush Circle submit call site.
- Keeps one thread-safe `BrushCursorModel` as the source of truth for visibility, fresh-ray-cast validity, camera suppression, world hit, normal, and radius. The DrawOverride snapshots it in `prepareForDraw()` and submits one surface-oriented circle in `addUIDrawables()`.
- Marks both the temporary helper transform and shape as non-writing, hides them from the Outliner, disables user selection and bounding-box display, avoids Maya's undo command path, and deletes them on tool cleanup or plug-in unload.
- Exposes brush radius through the context's `-radius` / `-r` Query/Edit flag.
- Uses explicit `Hidden -> Hover -> RadiusAdjust -> Hidden` and `Hover -> Hidden(camera) -> Hidden -> Hover` transitions. B Release and camera exit never restore an old hit; only a fresh pointer ray cast can return to Hover.
- Invalidates hover feedback during Maya's alternate camera context and when the pointer exits the viewport, preventing stale world-space Brush Cursors after tumble, pan, or dolly.
- Supports Maya-style `B` + left/right drag radius adjustment. Radius mode is owned by the C++ Context; a reload-safe, active-tool-only PySide2 event filter forwards B press/release without changing Maya hotkeys.
- Installs and removes the B-key runtime from `MPxContext::toolOnSetup()` and `toolOffCleanup()`. Python activation, Maya Tool History, and direct `setToolTo()` reactivation therefore share the same lifecycle, with exactly one filter while active and zero while inactive.
- Keeps Raw Stroke input separate from Processed Stroke output. Release performs distance-based endpoint trimming, multi-sample direction estimation, tangent-plane projection, and light direction smoothing without appending a release sample.
- Saves the active selection at tool activation, retains the resolved mesh `MDagPath` as the internal target, and clears Maya's active selection so the standard selection highlight does not compete with Stroke feedback.
- Restores the original valid selection when the Context is switched off, deleted, or unloaded. Deleted targets and invalid selection entries are skipped safely.
- Registers one temporary `directionalRetopoTargetDisplayShape` with a persistent `MPxDrawOverride`. It is independent from the high-frequency Brush Cursor node and owns the Target Wireframe's only batched `lineList()` submit call.
- Builds the low-opacity gray Target Wireframe cache when the target is acquired. Selective mesh-geometry callbacks invalidate vertex/topology data, while transform-only changes update only world-space points without re-reading edge connectivity.
- Keeps the Target DrawOverride clean during Hover, B adjustment, Stroke sampling, Brush Cursor refresh, and camera navigation. Those events reuse its retained draw data and perform zero Target topology scans, edge enumerations, or cache rebuilds.
- Keeps finalized `ProcessedStroke` data separate from the transient active-stroke and direction visualization. Release, a new stroke, camera alternate context, and tool cleanup clear only the appropriate transient feedback.
- Adds a provisional `RegionPreviewCalculator` that caches face bounds and adjacency at target acquisition and produces a replaceable Face ID set from current stroke samples and radius.
- Draws the provisional Region Preview as one batched, low-alpha `MUIDrawManager::mesh(kTriangles, ...)` call while dragging.
- Builds `MeshTopologyCache` once when the target is acquired: world-space vertex positions and edge lengths, edge-to-vertex/face data, face-to-vertex/edge/adjacency data, and vertex-to-edge adjacency. Release refreshes positions and lengths for current geometry/transform without re-reading every connectivity relation.
- Runs the formal `PaintRegionSolver` only after Mouse Release and only from the finalized `ProcessedStroke`.
- Uses processed hit-face vertices as sources for a radius-bounded, multi-source Dijkstra search over mesh edges. Edge costs use world-space edge length, so nearby but topologically distant folded surfaces are not selected through empty space.
- Retains linear vertex and face influence arrays, direct-hit Core faces, face-connected Core components, a configurable two-ring Transition band, and the Complete Region per component.
- Extracts Boundary edges with inside/outside face IDs and original-open-boundary flags, then orders them into closed loops or open/ambiguous finite chains without attempting non-manifold repair.
- Replaces the provisional fill on Release with batched warm Core fill, lighter Transition fill, and a thin Boundary line. Final Region geometry is independent from Brush Cursor and Target Display caches and is not dirtied by Hover.
- Prints concise Core/Transition/Boundary/component counts, bounded-search work, and measured solve time once per completed stroke.
- Builds a stable world-space face center, normal, and tangent basis from cached mesh geometry for every Region face.
- Converts Processed Stroke directions into face-local `cos(4 theta), sin(4 theta)` constraints, making 90-degree rotations and reversed strokes equivalent Quad orientations.
- Parallel-transports directions across each shared edge by its signed dihedral rotation before comparing or smoothing neighboring face fields.
- Applies iterative, soft-constrained 4-RoSy smoothing over Core and Transition while increasing existing-topology orientation guidance toward the Boundary.
- Stores normalized tangent `U`, perpendicular `V`, normal, paint constraint strength, topology guidance strength, and validity for every source polygon face.
- Builds Manual or Auto Density fields with Target Edge Length as the source of truth. Auto mode samples Boundary-outside faces, rejects invalid/outlier lengths, and uses a robust median with explicit fallbacks.
- Blends Manual density toward surrounding topology through Transition and Boundary faces and stores isotropic `scaleU`/`scaleV` values for later anisotropic extension.
- Draws sampled Direction crosses with one batched `lineList()` and optional Density gradient points with one batched point-mesh submission. Independent toggles and glyph limits keep debug feedback controllable.
- Exposes a read-only `QuadSolveInput` view containing `PaintRegionData`, `DirectionFieldData`, and `DensityFieldData`.
- Uses Maya's cached `MFnMesh::getTriangles()` result to triangulate every Complete Region component independently while retaining local-vertex to source-vertex, patch-triangle to source-face, source-face to patch-triangle, and source/patch Boundary mappings.
- Adapts each triangle's projected Phase 3 `U` direction to AutoRemesher external guidance and maps Target Edge Length to the upstream solver's actual global/per-face scaling terms in one adapter layer.
- Runs parameterization and quad extraction only after Mouse Release. Parameter UVs are retained for diagnostics; Hover, camera navigation, and B-drag never invoke the solver.
- Validates arbitrary polygon output for finite coordinates, valid/non-repeated indices, area, manifold edge use, source-surface proximity, and Boundary loops before accepting it for Preview.
- Retains per-result-vertex nearest source triangle/source face/distance and Source/Result Boundary count, length, nearest-distance, orientation, and loop diagnostics for future Boundary Stitching.
- Registers at most one temporary `directionalRetopoQuadPreviewShape` with its own persistent `MPxDrawOverride`. A cached, unique-edge `lineList()` is rebuilt only for a new solve result or visibility change, never by `MPxContext::drawFeedback()`.
- Builds `DirectionalRetopoAutoRemesherCore` as an offline static library from the pinned solver-only source snapshot. It uses Maya 2024.2's TBB runtime and introduces no Qt or additional third-party DLL.
- Centralizes Target, Region, Brush, Stroke, and Direction colors, opacity, line widths, surface offsets, and Maya depth priorities in `ViewportVisualizationSettings`.
- Requests coalesced Viewport 2.0 refreshes with `M3dView::scheduleRefreshAllViews()` after visualization-state transitions instead of drawing from input callbacks.
- Logs tool activation, stroke start, and raw/processed final sample counts without logging every ray miss.

## Activate in Maya

Build the plug-in, start Maya 2024.2, and run this in the Script Editor's Python tab:

```python
import sys

scripts_path = r"C:\path\to\DirectionalRetopo\scripts"
if scripts_path not in sys.path:
    sys.path.insert(0, scripts_path)

from directional_retopo import tool
tool.activate(radius=1.0)
```

Before activation, select exactly one polygon mesh. The helper loads the local Release `.mll`, creates or reuses `directionalRetopoContext1`, applies the radius, and activates the tool. `toolOnSetup()` then installs the active-context-only B-key runtime regardless of whether activation came from this helper, Tool History, or `setToolTo()`. Activation retains that mesh internally and clears Maya's active selection; painting does not require the mesh to remain selected.
It also sources the `DirectionalRetopoContextProperties.mel` and
`DirectionalRetopoContextValues.mel` hooks required by Maya's Tool Settings window.
Changing tools, calling `tool.deactivate()`, or calling `tool.unload_plugin()` removes the event filter through the Context lifecycle. No Maya hotkey is created or modified. During debugging, `directional_retopo.runtime.filter_count()` returns `1` while the tool is active and `0` otherwise.

The radius can be changed or queried later:

```python
tool.set_radius(2.0)
print(tool.query_radius())
```

Direction/Density settings are available without changing Maya hotkeys or Tool lifecycle:

```python
tool.set_density_mode("Auto")       # or "Manual"
tool.set_manual_target_edge_length(1.0)
tool.set_edge_length_scale(1.0)     # 0.5 finer, 2.0 coarser
tool.set_topology_blend_width(2)    # 0-10 shared Direction/Density rings
tool.set_field_visualization(show_direction=True, show_density=False)
tool.set_quad_preview_visibility(True)
tool.set_quad_preview_layers(show_raw=False, show_conformed=True)
tool.set_quad_preview_layers(
    show_source_boundary=True,
    show_result_boundary=True,
)
```

The same Radius, Density Mode, Manual Target Edge Length, Edge Length Scale,
and Topology Blend Width values are available from Maya's standard Tool
Settings panel. Numeric edits rebuild an existing valid Preview only when the
field commits its value; Hover does not run the solver. Auto mode disables the
Manual Target Edge Length control. The reset button restores defaults defined
by the C++ context.

## Phase 4.5 surface conformation

The pinned AutoRemesher output is retained as an immutable orange Raw Preview.
`SurfaceConformer` builds a local triangle/adjacency cache for the originating
Paint Region component, projects generated vertices only to that component,
anchors generated boundary vertices to its source boundary chains, and uses
tangential relax followed by local reprojection. A guard restores the initial
projected state if relaxation loses more than five percent area. The normal
cyan Preview uses the Conformed result; both layers can be toggled independently.

Each solve records Source/Raw/Conformed area and bounding boxes, Source average
and median edge length, Raw surface distance, projection distance, Conformed
surface distance, and conformation time. Distance thresholds of 5% mean and
20% maximum of Target Edge Length are quality warnings, not automatic
area-ratio pass/fail rules.

## Phase 4.6 curvature density and boundary conformation

Auto Density now retains the robust surrounding-topology median as its base,
then measures local normal variation from shared-edge dihedral angle divided by
world-space edge length. A target edge is refined only where the estimated
normal turn would exceed four degrees. The final Auto value is the smaller of
the scaled surrounding target and the curvature target; a five-times maximum
refinement factor plus an absolute minimum target length prevent unbounded Quad
counts. Flat faces retain the surrounding density, while chest/shoulder and
cloth-wrinkle regions receive local refinement. Manual mode keeps its existing
requested Target Edge Length and Transition behavior.

The adapter reports effective target min/mean/max and the number of
curvature-limited triangles. Surface diagnostics retain Raw and Conformed
positions, area ratios, source-surface distances, source edge statistics and
bounding boxes. The deterministic curved-patch diagnostic compares Scale 1.0,
0.5 and 0.2 and demonstrates that the adapter's effective spacing is exactly
the requested Target Edge Length; loss at coarse Scale comes from insufficient
parameter-grid/Quad sampling, not an inverted scale conversion.

`BoundaryConformer` extracts ordered Result boundary loops, pairs them with the
Phase 2/4 Source loops, determines closed/open state, seam and winding, and maps
each Result boundary vertex by normalized arc length onto its corresponding
Source curve. It preserves differing vertex counts and records per-vertex
normalized parameters and per-loop count, edge, arc-length and distance
diagnostics for Phase 5. It never splits, collapses or welds an edge. The Tool
Settings Source Boundary and Result Boundary checkboxes compare the yellow and
magenta cached VP2 line batches without rerunning the solver.

## Phase 4.7 ordered boundary correspondence

`OrderedBoundaryCorrespondence` now treats each Result boundary as one ordered
sequence. It exhaustively evaluates closed-loop cyclic seams and both traversal
directions, reorders only the Result loop metadata, then solves a strictly
monotonic Result-to-Source parameter path over sampled points on the ordered
Source polyline. Sharp Source corners are assigned to existing Result vertices
when the current Result count can represent them. Every mapped vertex retains
its Source edge ID, Source endpoint vertex IDs, edge parameter, original
normalized arc parameter and seam-relative unwrapped parameter.

The conformed Result vertices are evaluated from that ordered Source parameter;
independent nearest-edge projection is not used. Validation rejects reversed or
duplicated parameters, zero-length Result boundary edges, self-intersections,
and intersections with unrelated Source boundary intervals. If a Result edge
must shortcut a Source corner because the current Result topology has too few
vertices, a `RequiredBoundaryAnchor` records the Source vertex, Result edge,
corner angle and shortcut distance for Phase 5 without inserting a vertex or
changing a polygon.

Tool Settings can display Source/Result boundaries, near-zero correspondence
lines, and red Required Split Anchor points independently. The Python equivalent
for the two Phase 4.7 debug layers is:

```python
tool.set_boundary_debug_visualization(
    show_correspondence=True,
    show_anchors=True,
)
```


## Phase 5A / 5A.1 boundary-locked preview

`BoundaryLockedPatchBuilder` uses the ordered Source Boundary itself as the
final outer loop, so Density changes cannot move, collapse, or shortcut Source
Boundary vertices and edges.

`TransitionCollarBuilder` aligns the fixed outer loop with the ordered inner
AutoRemesher loop and uses monotonic dynamic programming to prefer Quads while
placing only the Triangles required to absorb count, parity, or density changes.
The paint Core remains the responsibility of the pinned solver.

Boundary validation is topology-first and measures true non-adjacent segment
intersections in 3D instead of rejecting a valid curved loop because of one
dominant-axis projection. Branched/non-manifold inner boundaries and true 3D
crossings still fail safely.

The final outer boundary remains pinned during surface conformation. Raw inner
solver data is retained for diagnostics when final Collar construction fails;
the Maya Source Mesh remains read-only.
Hold `B`, press the left mouse button, and drag horizontally in the viewport:

- Drag left to reduce the radius.
- Drag right to increase the radius.
- Release the mouse to finish adjustment. No StrokeSample is generated by this gesture.

The equivalent MEL edit command is:

```mel
directionalRetopoContext -e -radius 2.0 directionalRetopoContext1;
```

## Maya GUI verification

1. Load `build\Release\DirectionalRetopo.mll` through Plug-in Manager or the Python helper.
2. Create a polygon sphere and select only that sphere.
3. Activate the tool with `tool.activate()`.
4. Confirm that Maya's standard selection highlight disappears immediately.
5. Confirm that the retained target has a thin, translucent gray custom wireframe that is visually weaker than the blue Stroke.
6. Move the cursor over the now-unselected target and confirm that the green brush circle follows the surface and its normal.
7. Left-drag over the mesh and confirm the cyan stroke and yellow tangent-direction line remain clearly visible over the target wireframe.
8. Release the mouse and confirm `[DirectionalRetopo] Region generated` plus Core, Transition, component, Boundary, Dijkstra, and solve-time statistics in Script Editor.
9. Confirm that the transient cyan stroke and yellow direction disappear after Release and the provisional fill is replaced by warm Core fill, lighter Transition fill, and a thin Boundary line.
10. Confirm that the mesh's vertices, edges, faces, and topology are unchanged.
11. Change the radius and confirm that the brush size and sample spacing respond.
12. Hold `B` and left-drag left/right. During adjustment, confirm that exactly one circle stays at the anchor hit while only its radius changes and no paint stroke is generated. On B Release, confirm that the count is zero until the next pointer move; after that move, confirm that exactly one current-position circle appears.
13. Tumble, pan, and dolly with the normal Maya camera gestures. Confirm that the Circle count is zero from camera start through camera end, that the pre-camera circle never returns, and that exactly one current-position circle appears only after the next valid pointer move.
14. Paint a stroke with a small release-end wobble. Confirm that the final direction is stable during the stroke and no old transient stroke returns later.
15. Move the pointer outside the viewport and confirm that the hover circle is cleared.
16. Switch to another Maya tool or call `tool.deactivate()`. Confirm that the custom visualization disappears and the original mesh selection is restored.
17. Reactivate, delete the target, and switch tools. Confirm that cleanup completes without an exception or invalid selection restoration.
18. Switch to another Maya tool and confirm that its normal B-key behavior is unchanged.
19. Test no selection, multiple selections, and a non-mesh selection; each must stop safely with a clear warning.
20. While the tool is active, confirm that `cmds.ls(type="directionalRetopoBrushCursorShape")` and `cmds.ls(type="directionalRetopoTargetDisplayShape")` each return one node. After deactivation, both must return zero nodes.
21. Switch to Move or Rotate, reactivate DirectionalRetopo from Tool History, and confirm B + Drag works without calling `tool.activate()` again. Repeat several times and confirm `directional_retopo.runtime.filter_count()` remains exactly `1` while active and `0` while inactive.
22. On a subdivided plane, paint away from the outer edge. Confirm Core is surrounded by Transition and the outer Boundary is a closed visual loop.
23. On a polygon sphere, paint across curvature. Confirm the region follows face connectivity continuously.
24. On a folded open sheet with nearby front/back portions, paint one side. Confirm the spatially close but surface-distant side is not included.
25. Paint near an open plane edge. Confirm generation succeeds and the Boundary reaches the original mesh boundary without a crash; this is retained internally as an open stitch chain.
26. Repeat with very small and large radii. Confirm a direct-hit face always remains Core and the large-radius solve reports finite bounded Dijkstra work.
27. On a grid plane, paint a straight stroke and confirm the cyan/green Cross glyph U-axis follows it while V remains perpendicular.
28. Repeat the same path in reverse and with a 90-degree-equivalent orientation; confirm the Cross orientation remains equivalent without 180-degree flips.
29. Paint a curved stroke on a sphere and confirm Cross glyphs remain tangent and change smoothly across curved faces.
30. Confirm the field reaches Transition and is guided softly, not absolutely, toward existing edge orientation at the Boundary.
31. In Auto mode, confirm the reported median is close to the surrounding grid edge length. Add one extreme reference edge and confirm the median remains stable.
32. Switch to Manual mode, set a visibly different target length, repaint, and confirm the reported Core range reflects it while Transition blends toward the Boundary reference.
33. Enable Density display and confirm the sampled point gradient appears; independently toggle Direction and Density displays off and on.
34. Paint a straight stroke on a sufficiently subdivided plane. Confirm the Script Editor reports Patch build, Parameterization, Quad extraction, Validation, and Total timing once on Release.
35. Confirm a bright cyan/green Quad Preview appears, its main axes broadly follow the stroke Cross Field, and `cmds.ls(type="directionalRetopoQuadPreviewShape")` returns exactly one node while active.
36. Repeat the path in reverse, then paint a 90-degree path. Confirm reverse orientation remains equivalent while the 90-degree case rotates the preview axes.
37. In Manual density mode, compare a small and large Target Edge Length on the same region. Confirm the smaller value generally produces more polygons and the larger value fewer polygons.
38. Switch to Auto density on a uniform grid and confirm the reported target length follows the surrounding topology approximately.
39. Test a curved surface, an n-gon-containing target, a region touching an open mesh edge, multiple disconnected components, and a tiny region. Confirm each component succeeds independently or reports a safe diagnostic, never crashes, and never modifies the source mesh.
40. Hide and show the retained Preview with `tool.set_quad_preview_visibility(False/True)` and confirm this does not re-run the solver.
41. After deactivation, confirm Brush Cursor, Target Display, and Quad Preview shape counts are all zero and the original selection is restored.
42. Open Maya Tool Settings while DirectionalRetopo is active. Confirm Radius, Density Mode, Manual Target Edge Length, Edge Length Scale, and Topology Blend Width reflect context queries; Manual length must gray out in Auto mode.
43. Compare Topology Blend Width 0, 2, and 5 on the same valid stroke. Confirm each committed edit rebuilds Region/Direction/Density/Quad Preview and broadens the transition without a Hover-time solve.
44. Enable Raw Solver Result and Conformed Result together. On a sphere, chest/shoulder, or inflated cloth patch, confirm orange Raw and cyan Conformed differ where Raw shortcuts curvature, while cyan remains on the painted component and does not jump to a nearby body layer.
45. Switch to another tool and reactivate from Tool History. Confirm Tool Settings values persist, B-drag remains synchronized with Radius on the next Tool Settings refresh, and exactly one Brush/Target/Quad drawable exists.
46. In Auto / Edge Length Scale 1.0, compare a plane, sphere, chest/shoulder and wrinkled cloth. Confirm the Density log reports zero or little curvature refinement on the plane and local refinement on curved areas; the Conformed Preview must not show the coarse-patch shrink seen before Phase 4.6.
47. Enable Source Boundary and Result Boundary in Tool Settings. Confirm yellow and magenta curves overlap after conformation even when their reported vertex counts differ, and confirm the Script Editor reports seam-independent winding, closed/open state, arc length, count delta and near-zero conformed boundary distance.
48. Compare Auto Scale 1.0, 0.5 and 0.2. Confirm effective Target Edge Length and polygon count change as reported, while Scale 1.0 now receives the minimum local refinement required by curvature rather than refining flat areas globally.
49. Enable Boundary Correspondence and Required Split Anchors. Confirm Result boundary vertices remain on the yellow Source polyline in traversal order, no magenta edge crosses or runs backward, correspondence lines are effectively zero length after conformation, and only Source corners needing Phase 5 split propagation show red points.

Maya standalone can load and unload the plug-in, but it does not instantiate even built-in UI tool contexts. Brush interaction and Viewport 2.0 feedback therefore require the Maya GUI verification above.

## Source layout

```text
src/
|-- Plugin/    Shared command and plug-in names
|-- Tool/      MPxContext, MPxContextCommand, and MPxToolCommand
|-- Brush/     Brush settings and accelerated mesh ray casting
|-- Field/     4-RoSy math, Direction/Density builders/data, and Quad input view
|-- Mesh/      Cached mesh graph and ordered Boundary extraction
|-- Paint/     Stroke processing, PaintRegion data/solver, and provisional Preview
|-- Remesh/    Local patch conversion, AutoRemesher adapter, result data/validation
|-- Viewport/  Persistent Target/Cursor/Quad Preview and batched debug feedback
`-- pluginMain.cpp

scripts/
`-- directional_retopo/   Maya loading and activation helper
```

## Not implemented in Phase 5A.1

- Source Mesh vertex movement, edge split/collapse, face deletion, or replacement
- Welding the Preview into the Existing Mesh
- Undoable final topology commit
- Background-thread solving

The registered `DirectionalRetopoToolCommand` remains the integration point
for future one-stroke/one-undo scene changes. Phase 5A.1 is still a read-only
Preview and does not submit topology edits to Maya.

## Planned next phase

A later phase can consume the fixed Source Boundary, ordered inner Boundary,
Transition Collar diagnostics, Triangle reasons, and retained source mappings
to implement a validated, undoable Source Mesh replacement and weld. None of
those source-mesh modifications occur in this snapshot.
