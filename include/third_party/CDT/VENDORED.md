# Vendored: CDT (Constrained Delaunay Triangulation)

- Source: https://github.com/artem-ogre/CDT
- Version: 1.4.5 (tag `1.4.5`, commit `2068d015b9db3c92481e869b0c1f669b96a1d70a`)
- License: Mozilla Public License 2.0 (see `LICENSE` in this directory) — permissive, no conflict with this project's AGPLv3 licensing.
- Header-only distribution (`CDT/include/`), used as-is with no modifications.
- Not available as a Conan package (checked conancenter and cura-conan2, no recipe found) — vendored directly instead.
- Pulled in to support `src/vbct/` (see its own provenance note), which uses `CDT.h` for Stage 3's constrained Delaunay triangulation.
