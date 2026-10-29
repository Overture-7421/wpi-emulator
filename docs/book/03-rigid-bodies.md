# 3. Rigid-body simulation for robot programmers

## 3.1 Objectives

After this chapter the reader can choose a body type, calculate a cube's density,
and design collision filters for a plate sensor. No knowledge of Box3D's
internal solver is required.

## 3.2 Bodies and shapes

**Definition 3.1 (rigid body).** A body has one pose and velocity. Its collision
shapes move together without deformation. A static body does not move; a
kinematic body follows a prescribed velocity; a dynamic body responds to forces
and contacts.

The carpet and walls are static. Cubes are dynamic. A plate is kinematic because
the game prescribes its height. The plate can push a cube, but the cube's weight
does not determine the plate's motion through a mechanical hinge. That modeling
decision is classified in F-07 of [Appendix C](appendix-c-conformance.md).

A box shape uses half-extents, while a field visual uses full dimensions.
Confusing these produces an eightfold error in volume. For half-extents
$(h_x,h_y,h_z)$, volume is $8h_xh_yh_z$ and density is mass divided by that
volume. The manual's nominal 13 by 13 by 11 in cube and approximately 1.5 kg
mass [1, section 3.8] give half-extents
$(0.1651,0.1651,0.1397)$ m and density about 49.24 kg/m³.

The inertia of a uniform box about its centre follows, for example,
$I_{xx}=m(h_y^2+h_z^2)/3$. The backend computes mass properties from shape
density; the plugin need not repeat that calculation. The equation remains
useful when diagnosing an object whose rotational response is implausible.

## 3.3 Materials and contacts

Friction resists tangential sliding; restitution controls normal rebound.
The [physics adapter](../reference/physics.md) mixes friction by geometric mean
and restitution by maximum. With cube friction 0.5 and carpet friction 0.8, the
mixed coefficient is $\sqrt{0.4}\approx0.632$. These coefficients describe
the model, not measured 2018 field materials.

For a single box on an ideal inclined plane, the static sliding threshold is
$\theta=\arctan\mu$. This calculation concerns sliding only. A stack can also
tip or lose contact, so it does not establish that the initial pyramid is
stable. Chapter 7 tests the actual stack in the selected backend.

## 3.4 Filtering and sensing

Each shape has a category and a mask. A pair is eligible only when both masks
accept the other category. `layer::Fuel` is the SDK's historical name for the
generic game-piece category; the cube uses that category without becoming a
sphere or inheriting another game's behavior.

A sensor detects overlap without producing a collision response. Plate sensors
accept game pieces and reject field structures. Sensor events must be enabled
on the sensor and visiting shapes. An overlap is only a candidate: a fast cube
passing through a sensor is not necessarily resting on the plate.

Sensors do not provide continuous collision detection. A narrow volume can be
crossed between samples. Chapter 8 therefore combines overlap with a footprint,
height and speed test, and Chapter 10 describes the corresponding sampled
interaction-zone model.

## 3.5 The engine boundary

The repository pins Box3D at v0.1.0 [7]. The plugin sees `IPhysicsWorld`, not
Box3D types. Tests may combine a real backend with the plugin, but production
plugin code includes only SDK and its own headers. This boundary permits a
physics adapter to change without rewriting game rules.

The SDK has field-anchored chassis constraints for mechanisms. It does not expose
a general arbitrary-body revolute joint API. These are different capabilities:
Chapter 18 examines chassis coupling for a climber without treating it as a
hinge for the scale plates.

## 3.6 Summary

Body type, geometry, material and filter each answer a different question.
The adapter supplies integration and contact response. Game rules still need
an operational definition of such terms as *resting on*.

## 3.7 Exercises

1. Calculate the mixed friction of two materials with coefficients 0.2 and 0.8.
2. Show the density error caused by supplying full cube dimensions as half-extents.
3. Design masks for a sensor that detects cubes but not the robot or carpet.

[Chapter 4](04-specification.md)
