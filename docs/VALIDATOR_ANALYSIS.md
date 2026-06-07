# Validator — Analysis (`smp report`)

A developer's guide to the physics-asset validator added by this branch: what it does, how
it is built, what it is good for, and where it stops.

> **One line.** `smp report` is an **offline, read-only** QA pass over a load order's FSMP
> physics assets. It scans every physics NIF and physics XML (or only the equipped gear),
> reports defects that would crash or misbehave in the physics engine, and writes a
> timestamped report to the SKSE log directory. It never modifies any file.

This branch ships **only the read/report path**. The matching _repair_ and _trim_ commands
(`smp fix`, `smp trim`) that act on the same findings are a separate, later addition; several
modules here intentionally carry their write-side helpers unused for that reason (called out in
the commit messages).

---

## What it checks

`ValidatePhysicsAssets` ([hdtAssetValidator.cpp](src/Validator/hdtAssetValidator.cpp)) drives a
phase-ordered pipeline. Each capability:

| Check                                     | What it catches                                                                                                                                                                                                              | Where                                                                                                                                                                              |
| ----------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Physics XML vs XSD**                    | unknown/misplaced elements, missing required attributes, bad enum values, out-of-range numeric values, broken key/keyref cross-references                                                                                    | [hdtXSDValidator.cpp](../src/Validator/Validators/hdtXSDValidator.cpp)                                                                                                             |
| **Physics XML vs Schematron**             | rule-based asserts/reports with XPath-style locations (e.g. value-range rules)                                                                                                                                               | [hdtSCHValidator.cpp](../src/Validator/Validators/hdtSCHValidator.cpp)                                                                                                             |
| **Runtime-accurate XML warnings**         | out-of-`[0,1]` "factor" values reported as the clamped effective value; redundant tags suppressed unless redundant against the _effective inherited template_                                                                | [hdtTemplateDefaults.cpp](../src/Validator/Utils/hdtTemplateDefaults.cpp) + `appendXmlViolationsToReport`                                                                          |
| **NIF physics-ref extraction**            | finds `"HDT Skinned Mesh Physics Object"` references; flags NIFs pointing at a **missing** XML, or carrying **multiple** physics blocks (only the first is used)                                                             | [hdtNIFPhysicsXMLExtractor.cpp](../src/Validator/Validators/hdtNIFPhysicsXMLExtractor.cpp)                                                                                         |
| **NIF skin-mesh integrity** (crash-class) | orphaned `NiSkinInstance` with no `NiSkinPartition` (runtime dereferences it → crash); partition triangle-copy mismatch; shape/partition count or triangle mismatch; non-permutation vertex map; triangle index out of range | [hdtNIFSkinMeshValidator.cpp](../src/Validator/Improvers/hdtNIFSkinMeshValidator.cpp), [hdtNIFOrphanedSkinImprover.cpp](../src/Validator/Improvers/hdtNIFOrphanedSkinImprover.cpp) |
| **`_0`/`_1` pair consistency**            | weight-variant body pairs that reference different physics XML, or a different number/order of physics blocks                                                                                                                | `validateNifPairConsistency`                                                                                                                                                       |
| **DefaultBBP validation**                 | `defaultBBPs.xml` `<map>` entries pointing at missing or invalid physics XML                                                                                                                                                 | `discoverDefaultBBPXMLs`                                                                                                                                                           |
| **LE-format detection**                   | warns that a Skyrim **LE** (pre-SE) NIF can't be used by FSMP and must be converted                                                                                                                                          | `isPreSESkyrimNif` in [hdtNIFBinaryIO.cpp](../src/Validator/Improvers/hdtNIFBinaryIO.cpp)                                                                                          |
| **Runtime structure (equipped)**          | on equipped gear: walks the live `NiNode` graph for null skin data, zero bones, null bone arrays, suspicious root scale                                                                                                      | [hdtNIFStructureValidator.cpp](../src/Validator/Validators/hdtNIFStructureValidator.cpp)                                                                                           |

---

## Architecture

A clean, layered, one-directional pipeline. Everything is **read-only** — no writers ship on
this branch.

```
  smp report  ──►  ValidatePhysicsAssets  (orchestrator + report writer)
                          │
            ┌─────────────┴──────────────┐
            │        Validators           │   XSD · SCH · NIF physics-ref · NIF structure
            └─────────────┬──────────────┘
                          │
            ┌─────────────┴──────────────┐
            │  Parser   (XSD · SCH · NIF) │   text/bytes → models
            └─────────────┬──────────────┘
                          │
        Schema models · NIF binary IO · Utils · Config   (leaf layers)
```

### Pipeline phases (full scan)

1. **Phase 0 — DefaultBBP**: parse `defaultBBPs.xml`, validate each referenced XML.
2. **Phase 2 — NIF discovery**: collect every `.nif`, binary-scan each for physics markers.
3. **Phase 2.5 — pair consistency**: check `_0`/`_1` reference the same physics data.
4. **Phase 3 — NIF-referenced XML validation**: XSD + SCH on each unique physics XML, with per-NIF context.
5. **Phase 3.5 — NIF structural validation**: binary skin-mesh integrity + orphaned-skin + LE detection.

Equipped-only (`gear`) collapses to: discover equipped armor/headparts → validate their XML →
runtime structural checks on the live `NiNode`.

### Design properties

- **Read-only / non-destructive.** No file is ever written except the report log. The "improver"
  modules present (binary IO, orphaned-skin, skin-mesh validator) ship whole, but only their
  read functions (`parseNif`, `countOrphanedSkinInstances`, `detectNIFSkinMeshIssues`,
  `isPreSESkyrimNif`) are reachable from `smp report`.
- **Schema-driven, not hardcoded.** Element names, enums, ranges, required attributes, and
  content models all come from `hdtSMP64.xsd` / `hdtSMP64.sch`, so the validator and the runtime
  agree on what is legal. Schemas load once behind a `std::once_flag`.
- **Parallel.** Discovery walks top-level directories with one `std::async` per directory and a
  native NTFS scan that bypasses the MO2 VFS when a `mods-dir` is configured; XML validation runs
  in parallel and is de-duplicated so a shared XML is validated once.
- **Background.** The console command runs the whole pass on a detached thread so the game is not
  blocked; a single in-flight guard prevents concurrent runs.

---

## Usefulness

- **Catches crash-class defects before load.** Orphaned `NiSkinInstance` blocks and out-of-range
  partition triangles crash the physics runtime; the report names the exact NIF and shape.
- **Authoritative XML linting.** Because it reads the same schema the engine documents, it tells
  authors precisely which tags are unknown, misplaced, out of range, or redundant — and is honest
  about runtime behaviour (factor clamping, effective-template redundancy).
- **Whole-load-order checks.** `_0`/`_1` pair drift and `defaultBBPs.xml` mistakes are caught,
  which a single-file linter would miss.
- **Fast on big setups.** The native mods-dir scan avoids the dominant MO2 VFS cost.

## Limitations

- **Read-only.** It diagnoses; it does not fix. Repair/decimation (`smp fix`, `smp trim`) are not
  on this branch.
- **SSE format only.** The binary skin-mesh checks understand Skyrim SE `BSTriShape`/`NiSkinPartition`.
  Skyrim LE meshes are detected and flagged, not analysed; unrecognised SSE vertex layouts are
  skipped (reported as warnings, not errors).
- **Needs the schema files.** XML validation requires
  `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/hdtSMP64.xsd` and `…/hdtSMP64.sch`. If they are absent,
  XSD/SCH validation is silently skipped (XML is treated as passing).
- **Equipped scope is runtime-bound.** `smp report gear` validates only what is currently worn on
  tracked skeletons, and uses human-readable item identifiers (not disk paths) because the NIF
  file path is not known at runtime.
- **Output is a flat log.** The report is human-readable text, not structured (JSON/CSV) data.

---

## Code map

- Entry point: `ValidatePhysicsAssets` — [hdtAssetValidator.h](../src/Validator/hdtAssetValidator.h) / [.cpp](../src/Validator/hdtAssetValidator.cpp).
- Console command: `smp report` handler in [main.cpp](../src/main.cpp).
- Config: `ValidationConfig` (just `modsDir` on this branch), loaded from `configs.xml` by [config.cpp](../src/config.cpp).
- Validators: [Validators/](../src/Validator/Validators/) · Parsers: [Parser/](../src/Validator/Parser/) · Schema models + nif.xml: [Schema/](../src/Validator/Schema/) · helpers: [Utils/](../src/Validator/Utils/).
