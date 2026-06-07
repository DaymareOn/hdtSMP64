# Validator — How to run `smp report` and read the results

`smp report` runs FSMP's physics-asset validator and writes a report file. It is **read-only**:
it never changes your mods. Use it to find physics XML / NIF problems in a load order before they
crash or misbehave in game.

---

## 1. Prerequisites

- FSMP installed and loading (you can run other `smp` console commands).
- **Schema files present** (needed for XML validation):
  - `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/hdtSMP64.xsd`
  - `Data/SKSE/Plugins/hdtSkinnedMeshConfigs/hdtSMP64.sch`
  - If these are missing, the run still works but XML is **not** schema-checked (treated as passing).
- Optional: a `mods-dir` in `configs.xml` (see [§5](#5-configuration)) for much faster scans on
  large load orders.

## 2. Running it

Open the in-game console (`~`) and type:

| Command                 | What it does                                                                               |
| ----------------------- | ------------------------------------------------------------------------------------------ |
| `smp report`            | Validate **all** physics assets in the load order. Full report.                            |
| `smp report gear`       | Validate **only the gear currently equipped** on tracked actors (you + nearby NPCs). Fast. |
| `smp report error`      | Full scan, but the report lists **errors only** (no warnings/info).                        |
| `smp report gear error` | Equipped-only, errors-only.                                                                |

(`smp help` lists every `smp` command and its usage, if you forget the syntax.)

The validation runs **in the background** — the game keeps running. You'll see a "started in
background" line immediately, then a completion line when it's done. Only one report can run at a
time.

On completion the console prints a summary and the **path to the report file**, e.g.:

```
[HDT-SMP] Report complete in 12.34s: 318 XML(s) found, 305 passed, 13 failed, 47 warning(s)
[HDT-SMP] Report written to: ...\Documents\My Games\Skyrim Special Edition\SKSE\hdtSMP64_validation_20260607_141233.log
```

## 3. Where the report goes

The report is written to the **SKSE log folder**:

```
Documents\My Games\Skyrim Special Edition\SKSE\hdtSMP64_validation_<timestamp>.log
```

(For GOG/VR the "Skyrim Special Edition" folder name differs accordingly.) A new timestamped file
is written every run, so old reports are kept.

## 4. Reading the report

A full report has these sections:

```
========================================
FSMP Asset Validation Report
Generated: 20260607_141233
========================================

== Summary ==
  Duration:      12.34s
  XMLs found:    318      <- unique physics XML files validated
  XMLs passed:   305
  XMLs failed:   13       <- XMLs with at least one blocking error
  NIF discovery: filesystem=41020, equipped=0, scan violations=2
  Warnings:      47
  Errors:        15

== Phase 0: DefaultBBP XML Validation ==      <- defaultBBPs.xml entries
== Phase 2: NIF File Discovery ==             <- how many NIFs scanned / physics-enabled
== Phase 2.5: NIF Pair Consistency Check ==   <- _0.nif / _1.nif agreement
== Phase 3: NIF-Referenced XML Validation ==  <- per-NIF: which XML, and its violations
== Phase 3.5: NIF Structural Validation ==    <- crash-class mesh defects

== Errors ==     <- flat index of every error, for quick triage
== Warnings ==   <- flat index of every warning
========================================
```

Start at the bottom **`== Errors ==`** / **`== Warnings ==`** indexes for a quick list, then scroll
up to the matching `Phase` section for context (which NIF/XML it came from). `smp report error`
prints just the Summary + Errors.

Each finding is tagged `[ERROR]` or `[WARNING]`:

- **`[ERROR]`** — will break or crash physics, or is invalid against the schema. Fix before shipping.
- **`[WARNING]`** — works, but is redundant, ignored, or silently adjusted at runtime. Safe to leave;
  worth cleaning up.

### Common errors and what to do

| Message (abridged)                                                                            | Meaning                                                                               | Action                                                                    |
| --------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| `NIF … references missing XML: <path>`                                                        | The NIF points at a physics XML that isn't installed.                                 | Install the XML, or fix the path baked into the NIF.                      |
| `<file>:<line>: <element> - <message>`                                                        | XSD/Schematron violation in a physics XML (bad attribute, enum, range, or cross-ref). | Edit the XML per the message.                                             |
| `… NiSkinInstance block(s) with no NiSkinPartition ref — would crash the physics runtime`     | Corrupt skinned mesh; the engine would dereference a null partition.                  | Re-export the mesh, or repair it (NIF repair is a separate FSMP command). |
| `BSTriShape[i] (…): … data corruption` / `triangle references a non-existent vertex`          | The mesh's skin partition is internally inconsistent.                                 | Re-export / repair the NIF.                                               |
| `_0.nif has physics data but the matching _1.nif … was not found or has no physics reference` | A weight-variant body pair is mismatched.                                             | Ensure both `_0` and `_1` reference the same physics XML.                 |
| `_0/_1 NIF pair reference different physics XMLs` / `different number of physics XML blocks`  | The pair will behave inconsistently between weights.                                  | Make the pair match.                                                      |

### Common warnings and what they mean

| Message (abridged)                                                                               | Meaning                                                                         |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------- |
| `Multiple "HDT Skinned Mesh Physics Object" blocks found (N); only the first is used`            | The NIF has extra physics references that the runtime ignores.                  |
| `<tag> is set to the effective inherited default value … can be removed`                         | The tag duplicates the inherited template default; deleting it changes nothing. |
| `<tag> is shadowed by later <tag> … can be removed`                                              | A later tag in the same block overrides this one.                               |
| `… is out of range …; runtime clamps this value to [0, 1], so the effective value will be '<v>'` | An out-of-`[0,1]` "factor" value; the engine clamps it.                         |
| `… is not allowed inside <…>; this tag will be ignored`                                          | An unknown/misplaced element the runtime ignores.                               |
| `Skyrim LE / pre-SE NIF (bsVersion <n>); FSMP requires SE-format meshes`                         | The NIF is Oldrim format. Convert it (e.g. SSE NIF Optimizer).                  |
| `vertex format not recognised by this tool`                                                      | An SSE mesh layout the structural check doesn't analyse; skipped, not an error. |

## 5. Configuration

Optional `<validation>` section in
`Data/SKSE/Plugins/hdtSkinnedMeshConfigs/configs.xml`:

```xml
<validation>
  <!-- Path to your mod manager's mods folder (MO2 mods/ or Vortex staging).
       When set, the full scan reads each mod directory natively instead of through
       the virtual file system — much faster on large load orders.
       Leave empty to scan Data/ through the VFS. -->
  <mods-dir>C:\Modlists\MyList\mods</mods-dir>
</validation>
```

`mods-dir` only affects the **full** scan (`smp report`). The equipped scan (`smp report gear`)
reads live data and ignores it. Config changes apply after `smp reset` or a game restart.

## 6. Troubleshooting

- **"all physics assets OK"** — no errors or warnings found. Nothing to do.
- **Validation is already running** — a report is in progress; wait for it to finish.
- **XMLs found: 0 / nothing validated** — no physics-enabled NIFs were discovered. Check that the
  mods are installed/enabled and (for the full scan) that `mods-dir` points at the right folder.
- **No XML `[ERROR]`/`[WARNING]` ever appears, even on known-bad XML** — the schema files
  (`hdtSMP64.xsd` / `.sch`) are missing; install them so XML can be checked.
- **Lots of "Skyrim LE / pre-SE NIF" warnings** — those meshes are Oldrim format; convert them to
  SE before using with FSMP.
