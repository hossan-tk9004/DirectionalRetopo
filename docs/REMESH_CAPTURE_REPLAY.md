# R3.5 RemeshInput Capture and Replay

R3.5 calibrates the deterministic solver harness without changing the remesh
algorithm. A capture contains the exact portable solver::RemeshInput built by
MayaRemeshInputAdapter, followed by an optional Maya solve expectation.

## File format

- Magic: DirectionalRetopoRemeshInput
- Version: 1
- Endian marker: 0x01020304
- Encoding: deterministic bounded binary, with all container sizes stored as
  unsigned 64-bit values and all floating-point values stored losslessly as
  IEEE-754 double bytes.
- Suggested extension: .drinput

The implementation rejects a wrong magic, unsupported version, endian mismatch,
oversized container/string, truncated input, and any deserialized input that
fails RemeshInput::valid().

The payload preserves the complete portable source snapshot (positions, normals,
IDs, edges, faces, triangles, adjacency, and cached triangulation), Region
components and ordered fixed boundaries, per-face Direction and Density fields,
and all RemeshSettings. After Maya's solve returns, the same file is rewritten
with status, failure code, component/vertex/polygon and quad/triangle/n-gon
counts, diagnostics, quality values, and deterministic topology signatures.

## One-shot Maya capture

Capture is disabled by default. After activating the tool, arm exactly the next
valid solver input with:

    from directional_retopo import tool
    path = tool.capture_next_remesh_input(
        r"tests\fixtures\captured\maya_plane_success.drinput"
    )
    print(path)

The pending path can be inspected with:

    print(tool.query_pending_remesh_capture())

The next completed stroke that reaches MayaRemeshInputAdapter::build() consumes
the path. The input is written before DirectionalRemeshSolver::solve() and the
Maya expectation is added after the solve. No later stroke is captured unless
the command is armed again.

Use redistributable primitives (for example a Maya plane or sphere) for captures
that will be committed to the public repository. Do not capture proprietary
character or garment geometry.

## Harness replay

With Maya's bin directory on PATH:

    build\Release\DirectionalRemeshSolverHarness.exe ^
      --replay tests\fixtures\captured\maya_plane_success.drinput

    build\Release\DirectionalRemeshSolverHarness.exe ^
      --captured-dir tests\fixtures\captured

Each capture is solved five times. Replay fails unless Maya and Harness agree on
top-level status/failure code and every component's status, failure code,
vertex/polygon counts, quad/triangle/n-gon counts, and topology signature.
Successful outputs additionally execute the hard-invariant validator: finite
vertices, fixed-boundary displacement at most 1e-9, zero boundary crossings,
zero non-manifold edges, zero zero-area polygons, and zero n-gons.

## Required calibration set

Before R4 begins, capture at least:

1. flat/simple successful region;
2. curved successful region using a redistributable primitive;
3. another success with a different stroke direction or density;
4. one currently reproducible failure;
5. a second failure with a different failure mode or setting.

Maya GUI interaction is required for this set. Maya standalone can only validate
plug-in registration/load/unload; it cannot validate interactive Context strokes.
R4 remains blocked until all five captures replay with parity.
