# FSMP strand-wig — author guide

How to ship a custom strand-hair wig. A wig is three things: a **groom** (the strand geometry), a
**config** (look + physics), and an **equip record** (the in-game item that turns the wig on).

The FSMP DLL simulates the guide strands; Community Shaders (Strand Hair feature) draws and lights
them. As an author you don't touch either DLL — you ship data files (and, for the equip path, a
tiny plugin).

---

## 1. The groom — `grooms/<name>.tfx`

The strand geometry is a **TressFX `.tfx`** file placed in
`Data/SKSE/Plugins/FSMPWig/grooms/`.

Coordinate convention FSMP expects:

- Positions are **offsets from the head origin**, in **Skyrim units** (a head is ~12 units across).
- Axes are **world-up**: **+Z is up**. The scalp sits near `(0, 0, +10)`; strands grow toward `-Z`.
- Each strand's **vertex 0 is the root** (TressFX marks it with inverse-mass `w = 0`). FSMP pins it
  to the scalp; the rest fall under gravity + collision.
- `numVerticesPerStrand` must be a power of two: **4, 8, 16, 32, or 64**.

FSMP converts these to head-local at bind time, so one groom works on any actor regardless of head
orientation. Use `<groom-scale>` in the config to rescale if your DCC exported in different units.

**No DCC?** `contrib/tools/make_sample_tfx.py` generates a valid reference groom and is a worked
example of the exact byte layout `loadTfx()` reads:

```
python contrib/tools/make_sample_tfx.py grooms/sample.tfx
```

Everything in the `.tfx` header is range-checked on load; a malformed or oversized file is rejected
and the wig falls back to the procedural cap (it never crashes the game).

---

## 2. The config — `wig.xml` and `wigs/<id>.xml`

Plain XML (the same reader as SMP's own configs). The global `wig.xml` holds defaults; per-wig and
per-actor overrides live in `wigs/`, named by 8-uppercase-hex formID:

| File | Applies to |
|------|-----------|
| `wigs/<worn wig item formID>.xml` | anyone wearing that wig item (highest priority) |
| `wigs/<actor formID>.xml` | that specific actor (e.g. `00000014` = player) |
| `wig.xml` | global default |

Each override **inherits** the global config and sets only the tags it wants to change.

Tags:

| Tag | Meaning |
|-----|---------|
| `strands` | guide strand count (1..4096) — *procedural only; ignored when `<groom>` is set* |
| `verts-per-strand` | beads per strand (2..64) — *procedural only* |
| `length` | base strand length, Skyrim units (1..500) — *procedural only* |
| `length-variation` | per-strand length spread, fraction (0..1) — *procedural only* |
| `stiffness` | shape retention toward the rest pose (0..1) |
| `damping` | velocity retained per substep (0..1) |
| `width` | ribbon half-width base, world units (× the CS radius slider) |
| `color-root` / `color-tip` | linear RGB at root / tip, as `x`/`y`/`z` attributes |
| `groom` | authored `.tfx` filename in `grooms/` (bare filename only) |
| `groom-scale` | uniform scale on the authored groom |
| `wig-armor-only` | *global only:* `true` = attach only to wig-armor wearers; `false` (default) = all SMP actors |

All values are clamped on load; unknown tags are ignored; a bad value fails closed to the default.
Filenames are sanitized — a `<groom>` with a path separator or `..` is rejected.

---

## 3. The equip record — the in-game wig item

This is the only part that needs the **Creation Kit** (or xEdit) — a DLL cannot author a plugin.

- Make an **Armor** record in the **hair slot** (biped slot **31 kHair** or **41 kLongHair**). This
  is exactly what existing wig mods (e.g. dint999) already are, so you can reuse one as a template.
- The item's **formID** is the key: ship `wigs/<that formID>.xml` so wearing the item selects your
  wig's config.
- Set `<wig-armor-only>true</wig-armor-only>` in `wig.xml` for true equip-driven behaviour — strand
  hair then appears only on actors wearing a hair-slot armor.
- The armor's own mesh can be an empty/near-empty scalp NIF; the visible hair is the strand render.

---

## 4. What you ship

```
Data/SKSE/Plugins/FSMPWig/
  grooms/yourwig.tfx              # the strand geometry
  wigs/<wig item formID>.xml      # its look + physics
MyWig.esp                         # the hair-slot armor (equip record)
+ any scalp NIF / textures the armor needs
```

Requires: hdtSMP64 (FSMP) with the strand-wig engine, and Community Shaders with the Strand Hair
feature. No ENB required.
