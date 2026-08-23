# Third-party dependencies

Do not place the Autodesk Maya SDK or DevKit in this directory. The Maya 2024.2 DevKit remains external to this repository and is referenced by `build_maya2024.bat` and CMake.

Phase 4 includes a pinned, solver-only AutoRemesher snapshot under
`third_party/autoremesher`. See `autoremesher/UPSTREAM.md` for the exact commit,
selected source dependency graph, excluded application/UI components, scale
semantics, and local integration changes. Repository-level license attribution
is in `THIRD_PARTY_NOTICES.md`.

The normal build is offline. Do not replace the snapshot with a moving-master
download, and do not add Maya SDK files, AutoRemesher application binaries, Qt,
or a second TBB runtime here.
