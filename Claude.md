# Claude Code Instructions

> **Note:** If you are the Cowork researcher session, ignore these instructions.

1. In the parent directory to this folder, you will find the project's auto-generated research files. Go to `Root/Instructions/` and read everything there.
2. You have read/write access to the whole directory.
3. If a Specification file has been generated, read it before starting. As you work, you may update it (following the versioning rules in `File Folder Permissions.md`).
4. If there is anything in `Lessons Learned/`, read it. If you find and overcome difficulties during development, add the solutions there.
5. In `Self-Reference/`, you will find the output of the research agent. Read and familiarize yourself with the index.
6. `Work Product/` contains outputs, but for you, this folder is for **finished releases only** — not drafts or in-progress work.

## FeatherPrint-specific notes

- This is a fork of UltiMaker CuraEngine. The active development branch is `featherprint-5.14`, tracking `origin/featherprint-5.14` on GitHub (https://github.com/Rszalay/FeatherPrint).
- Build/deploy environment is Windows-only (VS2026/MSVC, Conan, CMake+Ninja, CuraEngine 5.14.0 base, since v1.2.0; `featherprint-5.13` is frozen). Read `Self-Reference/04-build-toolchain-facts.md` and `05-installer-distribution-facts.md` before touching the build or deploy toolchain; the narrative behind them is in `Lessons Learned/build_lessons_learned.md` and `installer_distro_lessons_learned.md` (static Arcus linking, DLL exclusions, engine/frontend version locking, Inno Setup pitfalls).
- The current canonical spec is `../Specifications/FeatherPrint - Spec REV 4.14 - 261007.md`. Its **Open Items** table (under Implementation Notes) is the current list of known defects and loose ends. Follow the versioning rule in `File Folder Permissions.md` for any update: copy, don't edit in place.
- Start code orientation at `Self-Reference/01-architecture-and-pipeline.md` and `09-feature-code-map.md` (which function produces which feature, and where logic is duplicated).
- The repo holds two generators: FeatherPrint and FeatherPrint Corrugated (built on the vendored VBCT library in `src/vbct/`). They are mutually exclusive `infill_pattern` values (`featherprint`/`corrugated`). The merge is complete (v1.0.0, 2026-09-18); `FeatherPrint Unification Plan.md` in the repo root is now a historical record. Corrugated's own spec and docs live in the FeatherPrint Corrugated project folder; see `Self-Reference/10-corrugated-vbct.md`.
- **Never re-implement live-path geometry in a pre-pass.** If a pre-pass needs the same geometry the per-layer generator builds (the toolpath OML, the open-layer virtual ring, anchor placement), extract one function and call it from both. Copies drift: Corrugated hit this three times, and FeatherPrint's Flange/Former collision pairing missed a fix that `generate()` got.
- The end-user manual is `FeatherPrint User Manual/` in this repo, the only copy. Its workflow PNGs are rendered from the neighbouring HTML (headless Edge at 852 px wide, scale factor 2, header/legend/footer hidden).
