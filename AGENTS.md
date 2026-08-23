# DirectionalRetopo Development Rules

## Mandatory target

- Target Maya: Autodesk Maya 2024.2 (API version 20240200).
- Platform: Windows x64.
- IDE: Visual Studio Community 2022.
- Compiler/toolset: MSVC v143 x64. Do not switch to Visual Studio 2026 or v145.
- CMake generator: `Visual Studio 17 2022` with the x64 architecture.
- Build configuration: Release.

## Build and verification

- Use `build_maya2024.bat` from the repository root for normal builds. It initializes the Visual Studio 2022 developer environment, configures CMake, and builds Release.
- Run a Release build after every C++ or CMake change.
- Do not report a successful build until `build/Release/DirectionalRetopo.mll` actually exists and the build command has returned success.
- Treat warnings and errors separately. Resolve the first root-cause compiler or linker error, then rebuild until the `.mll` is generated.

## Repository boundaries

- Do not copy Maya headers, libraries, DevKit samples, archives, or other Maya SDK files into this repository.
- Keep Maya and DevKit paths external and reference them through CMake/build configuration.
- Do not add `build/`, generated Visual Studio/CMake files, compiler intermediates, or `.mll` binaries to Git.
- Keep `src/`, `scripts/`, redistributable source and license files under `third_party/`, `CMakeLists.txt`, `build_maya2024.bat`, `AGENTS.md`, `.gitignore`, `README.md`, and `LICENSE` eligible for Git tracking.
- Update `.gitignore` when new tools or dependencies introduce generated files. Do not ignore redistributable source or required license files merely because they live under `third_party/`.
- Before committing, inspect `git status`, confirm that build products and Maya SDK files are not staged, review newly generated files, and verify that no Autodesk or third-party redistribution-restricted files are included.
- If a generated or restricted file is already tracked, remove it from the Git index without deleting the working-tree file (for example, with `git rm --cached`).
- Keep the project on the Maya 2024.2 / VS 2022 / v143 target unless the user explicitly requests a migration.
- Preserve the Phase 1.7 `MPxContext`, selection-independent stored target, cached `TargetVisualizer`, accelerated mesh ray casting, Raw/Processed/Transient Stroke separation, distance-based endpoint trimming, provisional Face-ID-based Region Preview, and Viewport 2.0 feedback architecture. Use the local Maya 2024.2 DevKit headers and samples as the source of truth for Maya APIs.
- Do not change Maya's global selection colors. Save and clear the target selection on tool activation, ray cast against the retained target path, and restore valid original selection items during tool cleanup.
- Keep the active-context-only PySide2 B-key event filter scoped to key-state forwarding. Radius drag state and Stroke suppression belong to the C++ Context, and the filter must be removed when the tool stops.
- Phase 1.7 must never change the target mesh. `RegionPreviewCalculator` is replaceable visualization scaffolding, not the formal Phase 2 PaintRegion solver. Keep mesh edits and topology operations out of the brush/context path until explicitly requested.
- Do not add AutoRemesher, QuadParameterizer, QuadExtractor, direction/density solvers, quad generation, or boundary stitching until the corresponding development phase is explicitly requested.
- Phase 4 is explicitly authorized to use the pinned solver-only AutoRemesher snapshot through the DirectionalRetopo adapter. Keep its application/UI/Qt layers excluded, use Maya 2024.2's TBB runtime, and do not replace the pinned commit with a build-time download.
- Phase 4 remains read-only: preserve component-local source mappings and Preview results, but do not delete source faces, move vertices, stitch/weld Boundaries, reconstruct Transition topology, or replace the Target Mesh. Quad Preview must remain a persistent VP2 drawable outside `MPxContext::drawFeedback()` and must not be dirtied by Hover.
- Maya standalone can validate plug-in registration and load/unload, but interactive Context creation and Viewport feedback require Maya GUI testing. Do not claim GUI behavior was automatically verified when only standalone tests ran.
- Preserve Phase 4.5 Raw Solver and Conformed Quad results separately. Surface conformation must project only to the originating triangulated Paint Region component, use tangential relax plus reprojection, and must never edit the Maya source mesh.
- `PaintRegionSolverSettings::transitionRings` is the single C++ source of truth for Tool Settings `Topology Blend Width`; it controls Paint Region expansion and therefore both Direction and Density transition blending. Do not add a duplicate GUI-only width.
- Do not implement Boundary vertex-count matching, welding, source-face deletion, or topology replacement until Phase 5 is explicitly requested.
- Preserve Phase 4.6 curvature-aware Auto Density: surrounding median times Edge Length Scale is the base, shared-edge normal variation supplies a local geometric cap, flat regions must not refine unnecessarily, and the configured minimum target / maximum refinement clamp must prevent density explosions.
- Preserve ordered Boundary arc-length conformation and its normalized Result-to-Source correspondence, seam, winding, closed/open, count, length, and distance diagnostics. Different Source/Result vertex counts are intentional in Phase 4.6; do not split, collapse, rebuild, or weld them before Phase 5.
