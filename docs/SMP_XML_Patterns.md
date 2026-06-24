# SMP XML Patterns

A **pattern** is a reusable, parameterized macro for SMP physics XML. You define a shape once (a cape,
a cloth tissue, a chain) and then drop it in wherever you need it, instead of hand-writing the same
cluster of bones and constraints over and over.

At load time FSMP **expands** every pattern into the ordinary `bone` / `generic-constraint` / etc.
elements it stands for, *before* anything else looks at the file. The runtime physics loader, the XSD
validator, the Schematron rules, and the redundancy checker all see only the plain, expanded result —
so a pattern can do nothing a hand-written file could not, it just saves you the typing.

> **Scope (current):** patterns are **file-local** — you define and use them in the same physics XML.
> Sharing a pattern across mods (a global library other authors import) is planned but not yet
> available. See *Limitations* below.

---

## The two halves: define, then use

```xml
<pattern-default name="cape">
  <param name="prefix"/>            <!-- required: no default -->
  <param name="rows"  default="6"/> <!-- optional: used if the use site omits it -->
  <body>
    ... the bones and constraints, with ${placeholders} ...
  </body>
</pattern-default>

<pattern name="cape" prefix="NPC_Cape" rows="8"/>
```

- `<pattern-default name="...">` declares a pattern: a list of `<param>`s and a single `<body>`.
- `<pattern name="..." .../>` instantiates it. The attributes supply the parameters.
- This mirrors the existing `<generic-constraint-default>` / `template="..."` convention: `*-default`
  defines, the bare tag uses.

---

## Filling in the blanks: `${...}`

Inside a `<body>`, `${name}` is replaced with the value of a parameter or a loop variable. It works in
both attribute values and text:

```xml
<bone name="${prefix}_${i}">${prefix} tail</bone>
```

You can also do **bounded index arithmetic** on an integer (a loop variable, or a parameter whose value
is a whole number):

```xml
bodyA="${prefix}_${i-1}"   <!-- the previous index -->
bodyB="${prefix}_${i+1}"   <!-- the next index     -->
count="${rows-1}"          <!-- one fewer than rows -->
```

Only `+` and `-` by a whole number are supported. There is no general expression language (no `*`, no
conditions) — keep patterns simple and readable.

If you reference a name that does not exist (`${typo}`), or do arithmetic on something that is not a
number, expansion **fails with an error** and the item gets no physics. Mistakes are never silently
ignored.

---

## Repeating: `<repeat>`

`<repeat var="i" count="N" [from="K"]>` emits its body once for each index, binding `${i}` to
`K, K+1, …, K+N-1` (default `from="0"`). Nest two of them for a 2-D grid.

A **cape** is a single chain of bones:

```xml
<pattern-default name="cape">
  <param name="prefix"/>
  <param name="rows"/>
  <body>
    <repeat var="i" count="${rows}">
      <bone name="${prefix}_${i}"/>
    </repeat>
    <repeat var="i" count="${rows-1}" from="1">
      <generic-constraint bodyA="${prefix}_${i-1}" bodyB="${prefix}_${i}" template="link"/>
    </repeat>
  </body>
</pattern-default>

<pattern name="cape" prefix="NPC_Cape" rows="8"/>
```

A **tissue** (cloth) is a grid of bones whose constraints also cross diagonally:

```xml
<pattern-default name="tissue">
  <param name="prefix"/>
  <param name="rows"/>
  <param name="cols"/>
  <body>
    <repeat var="i" count="${rows}">
      <repeat var="j" count="${cols}">
        <bone name="${prefix}_${i}_${j}"/>
      </repeat>
    </repeat>
    <!-- diagonal cross links; stay in range with rows-1 / cols-1 -->
    <repeat var="i" count="${rows-1}">
      <repeat var="j" count="${cols-1}">
        <generic-constraint bodyA="${prefix}_${i}_${j}" bodyB="${prefix}_${i+1}_${j+1}" template="x"/>
      </repeat>
    </repeat>
  </body>
</pattern-default>

<pattern name="tissue" prefix="Skirt" rows="6" cols="4"/>
```

A complete, ready-to-read example lives next to this page at
[`examples/patterns_cape_tissue.xml`](examples/patterns_cape_tissue.xml).

---

## Patterns can use patterns

A pattern's body may instantiate another pattern. Parameters passed in are evaluated in the *outer*
scope (so you can forward a loop index), while the inner body sees only its own parameters. Nesting is
bounded — a pattern that (directly or indirectly) uses itself forever stops with a clear error instead
of hanging.

---

## Safety limits

Because XML comes from outside FSMP, expansion refuses to run away. Each of these turns a blow-up into a
clean error rather than a freeze or a crash:

| Limit | Default | Trips when |
|-------|---------|-----------|
| Recursion depth | 8 | patterns nest (or cycle) too deeply |
| Per-`<repeat>` count | 10 000 | a single loop asks for too many copies |
| Total elements | 50 000 | the whole file expands to too many elements |

---

## When something is wrong inside a pattern

Validator messages about generated content point you back at the **pattern and the line where you used
it**, for example:

```
linearDamping equals the template default [generated by pattern 'cape' used at line 42]
```

so you fix the pattern definition or the use, not the machine-generated XML you never wrote.

---

## Limitations (today)

- **File-local only.** A `<pattern-default>` is visible only within the file that declares it. A shared
  cross-mod pattern library (import a `cape` someone else published) is planned, along with the
  namespacing and versioning that safely sharing implies.
- **Range-level diagnostics.** A validator message attributes generated content to the pattern and the
  use-site line; it does not yet pin the exact character inside the definition.
- **No external-editor schema yet.** The shipped `hdtSMP64.xsd` does not list `<pattern>` /
  `<pattern-default>`, so an XML editor validating against it will flag them. FSMP itself expands them
  before its own validation, so in-game and the FSMP validator are unaffected.
