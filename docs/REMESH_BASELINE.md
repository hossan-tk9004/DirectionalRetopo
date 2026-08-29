# R3 Deterministic Legacy Baseline

`DirectionalRemeshSolverHarness` constructs `solver::RemeshInput` directly. It
does not start Maya, create a scene, use `MFnMesh`, create an `MPxContext`, or
simulate mouse input. Every fixture is solved five times and compared with a
quantized/canonical topology hash.

The public input/output contract is Maya-independent. The R2 legacy solver
implementation still uses `MPoint`/`MVector` internally, so this executable links
the Maya 2024.2 OpenMaya/Foundation import libraries and needs Maya's `bin` on
`PATH`. Removing that math/runtime dependency is future implementation work, not
an R3 algorithm change.

## Recorded baseline

This is deliberately a characterization baseline: a test passes when observed
behavior matches its declared expectation. `ExpectedKnownFailure` is not a solver
success and must become `ExpectedSuccess` only when a later redesign intentionally
fixes it.

| Fixture | Expectation | Recorded failure | Stable over 5 runs | Baseline observation |
|---|---|---|---|---|
| plane_fine | ExpectedKnownFailure | TransitionBuildFailed | yes | inner extraction exposes no ordered boundary |
| plane_coarse | ExpectedKnownFailure | TransitionBuildFailed | yes | initial extraction may be empty; retries expose no boundary |
| curved_fine | ExpectedKnownFailure | TransitionBuildFailed | yes | inner extraction exposes no ordered boundary |
| curved_coarse | ExpectedKnownFailure | TransitionBuildFailed | yes | inner extraction exposes no ordered boundary |
| small_region | ExpectedKnownFailure | QuadExtractionFailed | yes | extracted result is empty |
| blend_width_1 | ExpectedKnownFailure | TransitionBuildFailed | yes | both bounded attempts expose no boundary |
| odd_boundary | ExpectedKnownFailure | TransitionBuildFailed | yes | inner extraction exposes no ordered boundary |
| dense_boundary_coarse_core | ExpectedKnownFailure | TransitionBuildFailed | yes | coarse first attempt empty; bounded retries expose no boundary |
| chest_like_bump | ExpectedKnownFailure | TransitionBuildFailed | yes | curved inner extraction exposes no ordered boundary |
| cloth_fold | ExpectedKnownFailure | TransitionBuildFailed | yes | curved inner extraction exposes no ordered boundary |

These failures are precisely the legacy architecture baseline that the future
Source Transition Scaffold work must improve. R3 does not change AutoRemesher,
retry policy, collar topology, or expected outcomes.

## Success invariants and metrics

When a fixture becomes `ExpectedSuccess`, the harness additionally requires:

- finite output positions;
- fixed Boundary maximum displacement at most `1e-9`;
- zero Boundary crossings;
- zero unintended non-manifold edges;
- zero zero-area polygons;
- zero n-gons.

The summary reports quad/triangle/n-gon counts, surface mean/p95/max distance,
4-RoSy Core direction mean/max error, requested/actual Core edge length,
Boundary displacement/crossings, solve time, failure code, and canonical hash.
