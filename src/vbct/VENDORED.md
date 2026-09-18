# Vendored: VBCT

Source: `C:\Users\ricsz\source\CoWork Projects\Scratch\VBCT Cpp` (a separate, purpose-built Cowork project — see that project's own `VBCT - Spec REV 2.1 - 260826.md` for the full algorithm spec). That project remains read-only reference material; this is a copy of its `src/vbct/` sources (git commit `1a6aeb5`, "Port VBCT ... to C++, Stages 1-10"), not a live link.

Copied as-is, `.hpp`/`.cpp` kept together (not split into this repo's usual `include/`+`src/` convention) because VBCT's own files use same-directory quote-includes (`#include "pipeline.hpp"`) internally — splitting them would require rewriting those, which risks subtly breaking an already-validated 10-stage pipeline for no benefit.

Depends on vendored CDT (`include/third_party/CDT/`, see its own `VENDORED.md`). VBCT's own `tests/`, `bench/`, `tools/` (Catch2/nlohmann_json-based) are not vendored — only the `vbct_core` library sources, which don't need either dependency.

This project's own consumer code lives in `include/corrugated/VbctAdapter.h` / `src/corrugated/VbctAdapter.cpp`.
