# Current Drive base file

This document records the drivebase that was in the repository before the Team 1108-style direct
WPILib/REV implementation replaced YAGSL. It also identifies every value that must be checked on
the physical robot before normal operation.

## Active implementation

The active drivebase is now the adapted Team 1108 design in:

- `src/main/java/frc/robot/drive1108/DriveSubsystem.java`
- `src/main/java/frc/robot/drive1108/MAXSwerveModule.java`
- `src/main/java/frc/robot/drive1108/DriveConfigs.java`
- `src/main/java/frc/robot/drive1108/DriveConstants.java`

Unlike the old drivebase, it does not use YAGSL. WPILib performs kinematics and pose estimation,
REVLib directly configures the eight SPARK MAX controllers, Phoenix 6 directly reads the Pigeon 2,
and PathPlanner receives robot-relative chassis-speed callbacks.

## Preserved old CAN mapping

These values came from the old four YAGSL module files and were copied into `DriveConstants.java`:

| Module | Drive SPARK MAX / NEO | Steering SPARK MAX / NEO 550 |
|---|---:|---:|
| Front left | 7 | 8 |
| Front right | 5 | 6 |
| Rear left | 1 | 2 |
| Rear right | 3 | 4 |
| Pigeon 2 | 9 | — |

Important: the earlier full-robot repository also assigned shooter-right SPARK MAX ID 9. A Pigeon
2 and SPARK MAX cannot share CAN ID 9 on the same bus. The mechanism controller must be given a
different unique ID before restoring that mechanism code.

## Preserved old absolute-encoder configuration

Every old YAGSL module used:

```json
"encoder": { "type": "attached", "id": 0 },
"absoluteEncoderOffset": 0,
"absoluteEncoderInverted": true
```

The active direct implementation therefore currently assumes:

- A REV Through Bore absolute encoder is connected to each steering SPARK MAX.
- The encoder is Through Bore Encoder V2.
- All four encoders are inverted.
- All four forward offsets are zero radians.

The old zero values have been preserved as:

```text
Front left:  0 rad
Front right: 0 rad
Rear left:   0 rad
Rear right:  0 rad
```

These zero values are valid only if each encoder was hardware-calibrated with its wheel pointing
straight forward. If that was not done, measure and enter four software offsets in
`DriveConstants.java`. Do not apply both hardware and software correction for the same offset.

## Preserved old module PID and controller values

The previous YAGSL values were:

```text
Drive PID:    P=0.01, I=0, D=0, F=0, I-zone=0
Steering PID: P=0.01, I=0, D=0, F=0, I-zone=0
Heading PID:  P=0.4,  I=0, D=0.01
Path position PID: P=0.050, I=0, D=0
Path rotation PID: P=0.050, I=0, D=0
```

The Team 1108-style implementation presently uses Team 1108's starting control approach:

```text
Drive velocity PID: P=0.04, I=0, D=0
Drive velocity feedforward: kV = 12 / calculated theoretical free speed
Steering position PID: P=1.0, I=0, D=0
Path position PID: P=5.0, I=0, D=0
Path rotation PID: P=5.0, I=0, D=0
```

Both the old and new values are starting values, not verified final tuning. Tune steering first,
then drive velocity/feedforward, then PathPlanner translation and rotation.

## Preserved old drivebase dimensions and limits

These values came from the old YAGSL physical-properties and module-location files:

```text
Front-to-rear pivot spacing: 23 in = 0.5842 m
Left-to-right pivot spacing: 23 in = 0.5842 m
Each module coordinate:      ±11.5 in = ±0.2921 m
Wheel diameter:              3 in = 0.0762 m
Wheel radius:                1.5 in = 0.0381 m
Drive reduction:             5.08:1
Steering reduction:          46.42:1
Robot mass:                  87 kg
Wheel coefficient friction:  1.19
Drive current limit:         40 A
Steering current limit:      20 A
Old drive ramp rate:         0.25 s
Old steering ramp rate:      0.25 s
Nominal voltage:             12 V
Old Java maximum speed:      4.5 ft/s = 1.3716 m/s
```

The active direct implementation retains 1.3716 m/s as its initial safety cap. It does not
currently apply the old 0.25-second ramp rates. Add them to `DriveConfigs.java` if testing shows
they are required.

## Drivebase mathematics

### Module locations

WPILib uses +X forward and +Y left. With 23-inch square module spacing:

```text
Front left:  (+0.2921, +0.2921) m
Front right: (+0.2921, -0.2921) m
Rear left:   (-0.2921, +0.2921) m
Rear right:  (-0.2921, -0.2921) m
```

`SwerveDriveKinematics` converts chassis velocity `(vx, vy, omega)` into four wheel velocities and
angles using those module translations. Wheel speeds are desaturated to the configured maximum.

### Drive encoder conversion

For a 3-inch wheel and 5.08:1 reduction:

```text
wheel circumference = π × 0.0762 m
meters per motor rotation = (π × 0.0762) / 5.08
                          ≈ 0.04712 m/rotation
meters/second per RPM = 0.04712 / 60
                      ≈ 0.0007853 (m/s)/RPM
```

Using the NEO nominal free speed of 5,676 RPM:

```text
theoretical unloaded speed = (5676 / 60) × (π × 0.0762 / 5.08)
                           ≈ 4.46 m/s
```

The robot remains software-limited to 1.3716 m/s until it passes mechanical, encoder, pose, and
PID testing.

### Steering conversion

The Through Bore absolute encoder is converted to radians:

```text
one encoder rotation = 2π radians
steering error = optimized requested angle - corrected absolute angle
```

Continuous-input wrapping is enabled from 0 to `2π`, allowing a module to take the shortest path
across the zero boundary. WPILib module-state optimization may reverse wheel direction to avoid
turning the steering module more than 90 degrees.

### Pose estimation

`SwerveDrivePoseEstimator` combines:

```text
Pigeon 2 heading
+ four drive encoder distances
+ four absolute steering angles
+ verified module locations
= estimated Pose2d (field X, field Y, heading)
```

Timestamped vision measurements can be added through the two vision methods in
`DriveSubsystem.java`; no camera measurement is fused unless another subsystem calls them.

## PathPlanner values now aligned with the drive code

`src/main/deploy/pathplanner/settings.json` now uses:

```text
Wheel radius:      0.0381 m
Drive gearing:     5.08:1
Modeled max speed: 1.3716 m/s
Default auto speed: 1.0 m/s
Default accel:      1.0 m/s²
Track width:        0.5842 m
Module coordinates: ±0.2921 m
Robot mass:         87 kg
Drive motor:        NEO
Drive current:      40 A
```

The robot moment of inertia remains the preexisting estimate of 6.883 kg·m² and must be measured or
characterized. Robot width/length remain 0.9 m and should be changed to the measured bumper size.

## Controls now connected

```text
Left stick Y: field-relative forward/backward
Left stick X: field-relative strafe
Right stick X: angular velocity
Y button: invert all three driver inputs
Start button: zero heading to 0° blue / 180° red
X button held: X-lock the modules
```

The autonomous chooser is populated by PathPlanner and published as `Autonomous` on
SmartDashboard.

## Required physical-robot checks before driving

- [ ] Confirm all eight motor CAN IDs with REV Hardware Client.
- [ ] Confirm Pigeon 2 ID 9 with Phoenix Tuner.
- [ ] Resolve the historical shooter/Pigeon CAN ID 9 conflict.
- [ ] Confirm drive motors are NEOs and steering motors are NEO 550s on SPARK MAX controllers.
- [ ] Confirm the encoders are REV Through Bore Encoder V2. Change the preset in
      `DriveConfigs.java` if they are V1.
- [ ] Measure wheel diameter under robot weight.
- [ ] Confirm the actual drive reduction is 5.08:1.
- [ ] Measure both module-center spacings; confirm 23 inches.
- [ ] Point all wheels forward and verify each absolute angle is zero after a power cycle.
- [ ] Put the robot securely on blocks before its first enable.
- [ ] Verify each wheel's positive drive direction separately.
- [ ] Command 0°, +90°, -90°, and 180° on each steering module.
- [ ] Correct motor or encoder inversion immediately if steering moves away from its target.
- [ ] Confirm steering wraps from +179° to -179° by taking the short route.
- [ ] Confirm the Pigeon heading increases in WPILib's positive counterclockwise direction.
- [ ] Compile with the WPILib 2026 Java 17 toolchain using `./gradlew clean build`.
- [ ] First test robot-relative control at no more than 0.5 m/s.
- [ ] Then test field-relative control at headings 0°, 90°, 180°, and 270°.
- [ ] Reset pose and drive a measured 3 m; correct loaded wheel diameter if distance is wrong.
- [ ] Rotate exactly 360° and verify heading and pose behavior.
- [ ] Tune module control before attempting autonomous paths.
- [ ] Test a short straight autonomous at 0.5 m/s before using the 1.0 m/s default.
- [ ] Measure competition robot mass, bumper dimensions, and moment of inertia and update
      PathPlanner.

For the longer staged procedure, see `DRIVEBASE_1108_BRINGUP_CHECKLIST.md`.
