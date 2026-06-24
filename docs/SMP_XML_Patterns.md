# SMP XML Patterns

A **pattern** is a reusable, parameterized macro for SMP physics XML. You define a shape once (a cape,
a cloth tissue, a chain) and then drop it in wherever you need it, instead of hand-writing the same
cluster of bones and constraints over and over.

At load time FSMP **expands** every pattern into the ordinary `bone` / `generic-constraint` / etc.
elements it stands for, _before_ anything else looks at the file. The runtime physics loader, the XSD
validator, the Schematron rules, and the redundancy checker all see only the plain, expanded result —
so a pattern can do nothing a hand-written file could not, it just saves you the typing.

> **Two scopes:** you can define a pattern **in the same file** that uses it, or publish it in the
> shared **global patterns folder** so other mods can use it too. See _Sharing patterns across mods_.

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

A pattern's body may instantiate another pattern. Parameters passed in are evaluated in the _outer_
scope (so you can forward a loop index), while the inner body sees only its own parameters. Nesting is
bounded — a pattern that (directly or indirectly) uses itself forever stops with a clear error instead
of hanging.

---

## Sharing patterns across mods

To let other mods reuse your pattern, drop a file of `<pattern-default>`s into the shared folder:

```
Data/SKSE/Plugins/hdtSkinnedMeshConfigs/patterns/<yourMod>.xml
```

FSMP loads every `*.xml` there once at startup, and those definitions become visible to **every**
physics file — the author of a consuming file just writes `<pattern name="..."/>` and never copies
your definition.

**Namespace your patterns** with `author=` so they cannot clash with someone else's `cape`:

```xml
<!-- patterns/myMod.xml -->
<patterns>
  <pattern-default name="cape" author="myMod">
    <param name="prefix"/>
    <body> ... </body>
  </pattern-default>
</patterns>
```

A consuming file refers to it by its full `author.name`:

```xml
<pattern name="myMod.cape" prefix="NPC_Cape"/>
```

**Version your patterns** with `version=` when you need to change one without breaking mods that rely
on the old behavior. A use pins the version it wants:

```xml
<pattern-default name="cape" author="myMod" version="2"> ... </pattern-default>
<pattern name="myMod.cape" version="2" prefix="NPC_Cape"/>
```

A use with no `version=` takes the unversioned definition, or the sole version if there is exactly one;
if several versions exist and none is unversioned, the use must pin one (otherwise it is an error).

**Resolution and conflicts.** A file's own `<pattern-default>` overrides a shared one of the same full
name. If two shared files define the same full name, the one read later (by filename order) wins and a
warning is logged — so namespace with `author=` and you never have to think about load order.

---

## Safety limits

Because XML comes from outside FSMP, expansion refuses to run away. Each of these turns a blow-up into a
clean error rather than a freeze or a crash:

| Limit                | Default | Trips when                                  |
| -------------------- | ------- | ------------------------------------------- |
| Recursion depth      | 8       | patterns nest (or cycle) too deeply         |
| Per-`<repeat>` count | 10 000  | a single loop asks for too many copies      |
| Total elements       | 50 000  | the whole file expands to too many elements |

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

- **Diagnostics are line-precise, not character-precise.** A validator message points at the right
  _source line_ — the pattern use for generated content, the original line for hand-written content —
  but does not yet pin the exact character inside a pattern definition.
