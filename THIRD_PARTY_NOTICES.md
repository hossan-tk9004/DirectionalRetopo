# Third-Party Notices

DirectionalRetopo Phase 4 incorporates only the solver sources and header-only
dependencies listed below. Generated binaries, Maya SDK files, Qt, and unrelated
AutoRemesher application code are not redistributed.

## AutoRemesher solver core

- Project: AutoRemesher
- Upstream: https://github.com/huxingyi/autoremesher
- Pinned commit: `60b2fd4376850d83e04a5eccfa97096c2e0a6098`
- License: MIT
- License file: `third_party/autoremesher/LICENSE`

## Eigen 5.0.1

- Use: header-only dense and sparse linear algebra for AutoRemesher
- Primary license: Mozilla Public License 2.0
- Some individual files are available under the BSD license as identified in
  their source headers.
- License files are retained under
  `third_party/autoremesher/third_party/eigen/`.

## IsotropicRemesher AABB helpers

- Use: the minimal axis-aligned bounding-box tree used by QuadExtractor's
  smoothing/projection stage
- License: MIT
- License file:
  `third_party/autoremesher/third_party/isotropicremesher/LICENSE`

## Intel Threading Building Blocks

- Use: parallel loops and sorting in the AutoRemesher solver core
- License: Apache License 2.0
- Runtime: Maya 2024.2-supplied TBB 2020.3, interface 11103
- No TBB source or binary is redistributed by this repository.
- License notice retained at
  `third_party/autoremesher/licenses/TBB_LICENSE.txt`.
