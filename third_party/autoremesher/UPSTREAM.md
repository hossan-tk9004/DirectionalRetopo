# AutoRemesher Upstream Snapshot

## Source

- Repository: https://github.com/huxingyi/autoremesher
- Commit: `60b2fd4376850d83e04a5eccfa97096c2e0a6098`
- Commit date: 2026-08-17T23:33:58+10:00
- Retrieved: 2026-08-18
- Upstream license: MIT; see `LICENSE`.

The regular DirectionalRetopo build is fully offline. CMake does not fetch or
update this snapshot.

## Imported AutoRemesher components

Only the direct external-guidance parameterization and extraction path is
included:

- `constrainedleastsquares.{h,cpp}`
- `mixedintegerleastsquares.{h,cpp}`
- `quadparameterizer.{h,cpp}`
- `quadextractor.{h,cpp}`
- `surfacemesh.{h,cpp}`
- `meshseparator.{h,cpp}`
- `positionkey.{h,cpp}`
- `double.h`, `progress.h`, `vector2.h`, `vector3.h`
- corresponding `include/AutoRemesher` forwarding headers

The QuadExtractor also requires the following MIT-licensed helpers from the
upstream `thirdparty/isotropicremesher` directory:

- `axisalignedboundingboxtree.{h,cpp}`
- `axisalignedboundingbox.h`
- `double.h`, `vector2.h`, `vector3.h`

Eigen 5.0.1's `Eigen/` include tree is retained as the header-only sparse and
dense linear algebra dependency used by the solver. Its upstream license files
are retained next to the headers.

## Excluded components

The application, GUI, Qt, OpenGL shaders, resources, CLI/file I/O, automatic
FrameField pipeline, `Parameterizer`, `FrameField`, `SingularitySimplifier`,
isotropic remeshing pipeline, meshoptimizer, QtAwesome, QtWaitingSpinner, and
the bundled TBB source tree are not imported. libigl, Geogram, and Exploragram
are not dependencies of the selected current solver path.

## Runtime dependencies

- Eigen 5.0.1: header-only; snapshot included.
- TBB 2020.3 interface 11103: headers, import library, and DLL supplied by Maya
  2024.2. No TBB binary is copied into this repository or Maya's installation.
- MSVC C++ runtime: the same `/MD` runtime used by the Maya plug-in.

The solver core has no Qt dependency and introduces no additional DLL search
path or global `PATH` change inside Maya.

## Selected upstream APIs

`AutoRemesher::QuadParameterizer::parameterize()` accepts per-triangle external
guidance, a global dimensionless scaling, optional per-face scaling, and
optional U/V directional scaling. With a complete guidance array the upstream
automatic cross-field smoothing step is skipped. The solver still performs
quarter-turn representative brushing and bounded curl correction required by
its mixed-integer cover.

The effective world-space spacing in the upstream implementation is:

```text
scaling * patchAverageEdgeLength * faceScaling * directionalScaling
```

DirectionalRetopo therefore chooses a median target length as the global base,
uses `baseTarget / patchAverageEdgeLength` for `scaling`, and uses
`triangleTarget / baseTarget` for `faceScaling`. U/V scaling is `1.0` for the
current isotropic Phase 3 Density Field, avoiding double application.

`AutoRemesher::QuadExtractor` consumes the returned per-triangle UV values and
returns arbitrary polygon lists; DirectionalRetopo validates rather than
assuming every polygon is a quad.

## Phase 4.5 surface-fidelity audit

At this pinned commit, `QuadExtractor::extract()` calls
`smoothAndProject(5)` before its final topology cleanup passes.
`smoothAndProject()` applies a 0.5 world-space neighbor average to unlocked
vertices and projects each iteration back to the input triangle patch. The
subsequent split/collapse/merge/fan-conversion cleanup passes can create or
move final result positions, and there is no final projection after those
passes. DirectionalRetopo therefore preserves that final output as the Raw
result and performs component-local conformation in its adapter layer. No
upstream source was changed for Phase 4.5.

## DirectionalRetopo integration modifications

The upstream solver implementation files are otherwise unchanged. Seven
`include/AutoRemesher` forwarding headers used an incorrect one-level relative
path in the retrieved tree. Their include target was changed from `../src/...`
to `../../src/...` so it resolves within this preserved `include`/`src` layout:

- `Double`
- `MeshSeparator`
- `PositionKey`
- `Progress`
- `QuadExtractor`
- `Vector2`
- `Vector3`
