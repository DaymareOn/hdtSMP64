# hdtSMP64 XML Configuration Reference

This document describes the XML configuration options for hdtSMP64 physics definitions.

## Generic Constraints

Generic constraints use Bullet's `btGeneric6DofSpring2Constraint` for flexible spring-based physics. They can be configured with standard Hookean springs or non-linear Non-Hookean springs.

### Basic Constraint Options

```xml
<generic-constraint name="MyConstraint" bodyA="bone1" bodyB="bone2">
    <!-- Standard spring parameters -->
    <linearStiffness>100 100 100</linearStiffness>
    <angularStiffness>50 50 50</angularStiffness>
    <linearDamping>0.1 0.1 0.1</linearDamping>
    <angularDamping>0.1 0.1 0.1</angularDamping>
</generic-constraint>
```

---

## Non-Hookean Springs

Non-Hookean springs deviate from the standard linear force-displacement relationship (Hooke's Law: F = -kx). They allow springs to behave differently based on how far they are from equilibrium.

### Use Cases

- **Hair physics**: Soft movement near rest, stronger resistance at extremes
- **Cloth simulation**: Natural draping with realistic stretch limits
- **Body physics**: Softer jiggle physics with controlled limits

### Configuration Options

| XML Element | Type | Default | Description |
|-------------|------|---------|-------------|
| `linearNonHookeanDamping` | Vector3 | (0, 0, 0) | Reduces damping as spring approaches equilibrium |
| `angularNonHookeanDamping` | Vector3 | (0, 0, 0) | Reduces rotational damping near equilibrium |
| `linearNonHookeanStiffness` | Vector3 | (0, 0, 0) | Reduces stiffness as spring approaches equilibrium |
| `angularNonHookeanStiffness` | Vector3 | (0, 0, 0) | Reduces rotational stiffness near equilibrium |

### How It Works

The Non-Hookean algorithm modifies spring behavior based on normalized displacement:

```
normalizedError = |currentPosition - equilibrium| / range
factor = clamp(normalizedError, 0, 1)

effectiveDamping = baseDamping * (1.0 - nonHookeanDamping * factor)
effectiveStiffness = baseStiffness * (1.0 - nonHookeanStiffness * factor)
```

- **Factor = 0** (at equilibrium): Full damping/stiffness reduction applied
- **Factor = 1** (at limit): No reduction, behaves like standard Hookean spring
- **Values of 0**: Disabled (pure Hookean behavior)

### Example: Soft Hair Physics

```xml
<generic-constraint name="HairSpring" bodyA="hair_root" bodyB="hair_tip">
    <!-- Base spring parameters -->
    <linearStiffness>80 80 80</linearStiffness>
    <linearDamping>0.15 0.15 0.15</linearDamping>

    <!-- Non-Hookean: softer near rest, stiffer at extremes -->
    <linearNonHookeanDamping>0.5 0.5 0.5</linearNonHookeanDamping>
    <linearNonHookeanStiffness>0.3 0.3 0.3</linearNonHookeanStiffness>
</generic-constraint>
```

### Example: Cloth with Natural Drape

```xml
<generic-constraint name="ClothSpring" bodyA="cloth_anchor" bodyB="cloth_edge">
    <linearStiffness>50 50 120</linearStiffness>
    <angularStiffness>20 20 20</angularStiffness>

    <!-- More Non-Hookean effect on vertical axis for natural draping -->
    <linearNonHookeanStiffness>0.2 0.2 0.4</linearNonHookeanStiffness>
    <angularNonHookeanDamping>0.3 0.3 0.3</angularNonHookeanDamping>
</generic-constraint>
```

### Stability Warning

Non-Hookean values should typically be in the range [0.0, 1.0]:

- **0.0**: Disabled (Hookean behavior)
- **0.1 - 0.5**: Subtle effect, generally stable
- **0.5 - 0.8**: Noticeable effect, test carefully
- **> 0.8**: Strong effect, may cause instability
- **< 0.0 or > 1.0**: Not recommended, may cause physics explosion

---

## Vector3 Format

All Vector3 values are specified as three space-separated floats representing X, Y, Z components:

```xml
<linearStiffness>100.0 100.0 100.0</linearStiffness>
```

The coordinate system follows Skyrim conventions:
- **X**: Left/Right
- **Y**: Forward/Back
- **Z**: Up/Down

---

## Constraint Defaults

You can define default values that apply to all constraints:

```xml
<generic-constraint-default name="SoftSpring">
    <linearStiffness>50 50 50</linearStiffness>
    <linearDamping>0.1 0.1 0.1</linearDamping>
    <linearNonHookeanStiffness>0.2 0.2 0.2</linearNonHookeanStiffness>
</generic-constraint-default>

<!-- Uses SoftSpring defaults -->
<generic-constraint name="Hair1" template="SoftSpring" bodyA="a" bodyB="b"/>
<generic-constraint name="Hair2" template="SoftSpring" bodyA="c" bodyB="d"/>
```

---

## Additional Resources

- [Bullet Physics Documentation](https://pybullet.org/Bullet/BulletFull/classbtGeneric6DofSpring2Constraint.html)
- [hdtSMP64 Architecture](ARCHITECTURE.md)
