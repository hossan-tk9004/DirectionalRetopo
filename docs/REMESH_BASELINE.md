# R3.5 Captured Solver Baseline

`DirectionalRemeshSolverHarness` exercises the Maya-independent
`solver::RemeshInput` / `solver::RemeshResult` contract. The legacy solver
implementation still links Maya 2024.2 math/runtime libraries, so Maya's
`bin` directory must be on `PATH`, but replay does not create a scene,
`MFnMesh`, `MPxContext`, or mouse input.

This is a characterization baseline. `ExpectedKnownFailure` is a recorded
Maya result, not a passing remesh result. A fixture passes when deserialization,
Maya/Harness parity, success invariants, and five-run determinism match the
captured expectation. R3.5 does not change AutoRemesher, Transition Collar,
retry, density, or topology algorithms.

## Procedural fixtures

| Fixture | Origin | Density Mode | Expected | Observed | FailureCode | Quad | Triangle | Boundary Error | Direction Error | Density Ratio | Surface Error | Deterministic | Parity | Test Result |
|---|---|---|---|---|---|---:|---:|---|---|---|---|---|---|---|
| plane_fine | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| plane_coarse | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| curved_fine | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| curved_coarse | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| small_region | Procedural | Synthetic fixed | Known Failure | Failure | QuadExtractionFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| blend_width_1 | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| odd_boundary | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| dense_boundary_coarse_core | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| chest_like_bump | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |
| cloth_fold | Procedural | Synthetic fixed | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | n/a | PASS |

## Maya captured fixtures

All files are stored under `tests/fixtures/captured/`. The filename suffix
`topologylend1` is historical: the serialized source-of-truth value is
`topologyBlendWidth = 2` for all 12 captures.

For successful multi-component results, Quad/Triangle counts are aggregate
counts. Mean quality values are polygon-count weighted; maxima are the maximum
over components. Density is shown as requested/actual Core edge length followed
by actual/requested ratio.

| Fixture | Origin | Density Mode | Expected | Observed | FailureCode | Quad | Triangle | Boundary Error | Direction Error | Density Ratio | Surface Error mean/p95/max | Deterministic | Parity | Test Result |
|---|---|---|---|---|---|---:|---:|---|---|---|---|---|---|---|---|
| maya_Sphere_Radius10_manualDensity_EdgeScale05_topologylend1_a.drinput | Maya Capture | Manual | Success | Success | Success | 21548 | 646 | 0 / crossings 0 | 0 / 17.1887 deg | 0.5/0.5 (1.000) | 5.03189e-15 / 2.84217e-14 / 8.16351e-14 | yes | yes | PASS |
| maya_Sphere_Radius10_manualDensity_EdgeScale1_topologylend1_a.drinput | Maya Capture | Manual | Success | Success | Success | 8094 | 420 | 0 / crossings 0 | 0 / 17.1887 deg | 1/1 (1.000) | 5.56434e-15 / 2.92964e-14 / 6.70325e-14 | yes | yes | PASS |
| maya_Sphere_Radius1_autoDensity_EdgeScale1_topologylend1_a.drinput | Maya Capture | Auto | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |
| maya_Sphere_Radius1_manualDensity_EdgeScale05_topologylend1_a.drinput | Maya Capture | Manual | Success | Success | Success | 8951 | 594 | 0 / crossings 0 | 0 / 17.1887 deg | 0.523481/0.523481 (1.000) | 4.04862e-15 / 2.84217e-14 / 6.51223e-14 | yes | yes | PASS |
| maya_Sphere_Radius1_manualDensity_EdgeScale1_topologylend1_a.drinput | Maya Capture | Manual | Success | Success | Success | 558 | 72 | 0 / crossings 0 | 0 / 12.0061 deg | 1/1 (1.000) | 2.25705e-15 / 2.84217e-14 / 6.35529e-14 | yes | yes | PASS |
| maya_Sphere_Radius1_manualDensity_EdgeScale2_topologylend1_a.drinput | Maya Capture | Manual | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |
| maya_Torus_Radius10_autoDensity_EdgeScale2_topologylend1_a.drinput | Maya Capture | Auto | Success | Success | Success | 227 | 106 | 0 / crossings 0 | 0 / 17.1887 deg | 6.96598/6.85895 (0.985) | 8.57908e-16 / 7.10543e-15 / 3.17764e-14 | yes | yes | PASS |
| maya_Torus_Radius1_autoDensity_EdgeScale1_topologylend1_a.drinput | Maya Capture | Auto | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |
| maya_Torus_Radius1_autoDensity_EdgeScale2_topologylend1_a.drinput | Maya Capture | Auto | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |
| maya_plane_Radius1_AutoDensity_EdgeScale05_topologylend1_a.drinput | Maya Capture | Auto | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |
| maya_plane_Radius1_AutoDensity_EdgeScale1_topologylend1_a.drinput | Maya Capture | Auto | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |
| maya_plane_Radius1_AutoDensity_EdgeScale2_topologylend1_a.drinput | Maya Capture | Auto | Known Failure | Failure | TransitionBuildFailed | 0 | 0 | n/a | n/a | n/a | n/a | yes | yes | PASS |

Successful triangle percentages are:

- Sphere Radius10 Manual 0.5: 2.91%
- Sphere Radius10 Manual 1.0: 4.93%
- Sphere Radius1 Manual 0.5: 6.22%
- Sphere Radius1 Manual 1.0: 11.43%
- Torus Radius10 Auto 2.0: 31.83%

## Baseline summary

- Procedural: SUCCESS 0, KNOWN FAILURE 10
- Captured: SUCCESS 5, KNOWN FAILURE 7
- Auto captured: SUCCESS 1, FAILURE 6
- Manual captured: SUCCESS 4, FAILURE 1
- Capture deserialization: 12/12
- Maya/Harness parity: 12/12
- Five-run determinism: 12/12
- Parity mismatches: none
- Positive-control Success Validator: exercised by all five captured successes

## Success invariants

Every captured Success passed all of the following on every replay:

- all output positions finite;
- fixed Boundary maximum displacement at most `1e-9` (observed maximum: 0);
- Boundary crossing count 0;
- unintended non-manifold edge count 0;
- zero-area polygon count 0;
- n-gon count 0.

The stable/canonical hash includes status, failure code, component identity,
quantized vertex positions, and canonicalized polygon connectivity. Maya parity
also compares status, failure code, per-component vertex/polygon/quad/triangle/
n-gon counts, and captured topology signatures.

## Procedural versus captured input

The procedural fixtures intentionally remain a legacy characterization set, but
they are not calibrated replicas of Maya success inputs:

- procedural source meshes contain only 9-144 faces and usually mark the whole
  mesh as the remesh region; captures use 10,000-face Maya meshes with local
  regions of 48-619 faces;
- procedural boundaries are mostly one loop with 9-48 vertices; captures have
  one or two loops with 48-202 fixed vertices;
- procedural transition depth reaches 1-6, while captured data uses depth 2;
- procedural triangulation is hand-built, while captures preserve Maya
  triangulation (19,800 sphere triangles or 20,000 plane/torus triangles);
- procedural paint/topology direction weights are approximately
  0.33-0.62 / 0.25-0.44; captured means are approximately
  0.06-0.23 / 0.10-0.20;
- procedural density is spatially uniform; captures preserve Manual transition
  values or Auto/curvature-derived per-face values.

Because serialization round-trips are byte-identical and all captured Maya
outcomes replay with parity, this difference points to procedural fixture
construction rather than serialization loss or hidden Maya solver state.
Recalibrating procedural geometry is future fixture work and must not be used to
change the current R3.5 remesh algorithm.

## Capture behavior

A one-shot Manual capture serializes the portable input before interactive
safety checks. If its estimated output exceeds the normal interactive Manual
limit, only that explicitly armed capture bypasses the skip so Maya can record
the actual solver outcome. Normal interactive operations retain the safety
limit, and Tool Settings values are never modified.

No B-key/B+Drag fix, R4 Source Transition Scaffold, AutoRemesher update, or
source-mesh modification is part of this baseline.
