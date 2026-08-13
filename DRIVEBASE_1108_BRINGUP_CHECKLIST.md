# Team 1108-Style Drivebase Bring-Up Checklist

Do not put the robot on the floor or raise the speed limit until every off-floor check passes.
Keep the current YAGSL subsystem active until sections 1-5 are complete.

## 1. Confirm the mechanical and electrical configuration

- [ ] Measure center-to-center front/back module spacing. It must be 23.0 inches or update
      `WHEEL_BASE_METERS` in `DriveConstants.java`.
- [ ] Measure center-to-center left/right module spacing. It must be 23.0 inches or update
      `TRACK_WIDTH_METERS`.
- [ ] Confirm all four wheels are 3.0 inches in diameter. Use the measured loaded diameter for best
      odometry accuracy.
- [ ] Confirm the drive reduction is 5.08:1. If the pinion/module ratio differs, update
      `DRIVE_REDUCTION`.
- [ ] Confirm every drive motor is a NEO controlled by a SPARK MAX.
- [ ] Confirm every steering motor is a NEO 550 controlled by a SPARK MAX.
- [ ] Confirm every steering encoder is a REV Through Bore Encoder V2 connected to its steering
      SPARK MAX. If it is V1, change the preset in `DriveConfigs.java` before deploying.
- [ ] Confirm the Pigeon 2 is on the same CAN bus as the RoboRIO and has CAN ID 9.
- [ ] Inspect steering encoder and motor wires for enough slack to survive a full module rotation.

## 2. Confirm CAN IDs with REV Hardware Client and Phoenix Tuner

| Position | Drive SPARK MAX | Steering SPARK MAX |
|---|---:|---:|
| Front left | 7 | 8 |
| Front right | 5 | 6 |
| Rear left | 1 | 2 |
| Rear right | 3 | 4 |
| Pigeon 2 | 9 | — |

- [ ] Power one device at a time or use each vendor's identify function to verify the table.
- [ ] Resolve every duplicate CAN ID before deploying.
- [ ] Update all eight SPARK MAX controllers to the REV firmware required by REVLib 2026.
- [ ] Update the Pigeon 2 to a Phoenix 6-compatible firmware release.
- [ ] Verify all nine devices remain visible simultaneously with no CAN faults.

## 3. Calibrate the four absolute encoders

- [ ] Put the robot securely on blocks with every wheel clear of the floor.
- [ ] Mechanically point all four wheels exactly forward using a straightedge.
- [ ] In REV Hardware Client 2, open each steering SPARK MAX and use its Absolute Encoder utility.
- [ ] Zero/calibrate the Through Bore Encoder while that module is pointed forward.
- [ ] Power-cycle the robot without moving the wheels.
- [ ] Confirm every encoder still reports approximately zero when pointed forward.
- [ ] If hardware zeroing is not used, record each reading and enter the corresponding offset in
      `DriveConstants.java`. Do not use both a hardware zero and an extra software offset for the
      same error.

## 4. Install the correct development environment and compile

- [ ] Install the 2026 WPILib release and its Java 17 toolchain.
- [ ] Open the project using the 2026 WPILib VS Code launcher.
- [ ] Run `./gradlew clean build`.
- [ ] Fix every compilation or vendor-dependency error before connecting to the robot.
- [ ] Confirm REVLib, Phoenix 6, PathPlanner, and WPILib New Commands are installed.
- [ ] Do not remove YAGSL until the replacement drive has passed the complete checklist.

## 5. Perform off-floor module tests before activation

Create temporary diagnostics or expose module readings before making this subsystem the active
drive. Test one module at a time and command no more than 5-10% output.

- [ ] Positive drive voltage makes the top of the front-left wheel move toward the robot's front.
- [ ] Repeat the positive-drive check for front-right, rear-left, and rear-right.
- [ ] Increasing each steering encoder angle corresponds to the direction expected by the steering
      PID configuration.
- [ ] Command each module to 0 degrees; it points forward and holds without oscillating.
- [ ] Command each module to +90 degrees; it points left when viewed from above using WPILib's
      coordinate convention.
- [ ] Command each module to -90 degrees; it points right.
- [ ] Command 179 degrees followed by -179 degrees; it takes the short path across wraparound.
- [ ] Verify no steering motor runs continuously, chatters, or pulls against a hard stop.
- [ ] Verify drive current remains reasonable and steering current remains below the 20 A limit.

If a drive wheel runs backward, add the required drive-motor inversion to `DriveConfigs.java` only
after identifying whether the problem affects one module or every module. If steering moves away
from its setpoint, stop immediately and correct steering motor/encoder inversion before continuing.

## 6. Make the Team 1108-style subsystem active

Only do this after sections 1-5 pass.

- [ ] In `RobotContainer.java`, replace the `SwerveSubsystem` field with
      `frc.robot.drive1108.DriveSubsystem`.
- [ ] Replace the existing YAGSL default command with a command that calls:

```java
drivebase.drive(
    -MathUtil.applyDeadband(driver.getLeftY(), 0.05),
    -MathUtil.applyDeadband(driver.getLeftX(), 0.05),
    -MathUtil.applyDeadband(driver.getRightX(), 0.05),
    true);
```

- [ ] Ensure only one drive subsystem is constructed. Never construct YAGSL and the Team 1108
      subsystem together because they address the same eight SPARK MAX controllers.
- [ ] Add a button binding for `drivebase.zeroHeading()`.
- [ ] Deploy with the robot still on blocks and enable teleop at the lowest practical speed.

## 7. Verify chassis directions on blocks

- [ ] Forward stick: all four wheels point forward and rotate forward.
- [ ] Backward stick: all four rotate backward.
- [ ] Left strafe: all modules point left and drive in the correct direction.
- [ ] Right strafe: all modules point right and drive in the correct direction.
- [ ] Positive rotation: the four modules form the correct tangent directions and rotate the chassis
      counterclockwise under WPILib convention.
- [ ] Release the sticks: drive and steering controllers hold without oscillation.
- [ ] Disable the robot and confirm all motors stop immediately.

## 8. First floor test

- [ ] Clear a large test area and use a tether or immediately accessible disable control.
- [ ] Start with `MAX_SPEED_METERS_PER_SECOND` no higher than 0.5 m/s.
- [ ] Test robot-relative driving first by temporarily passing `false` for field-relative control.
- [ ] Verify forward, backward, strafe, and rotation individually.
- [ ] Verify the robot travels straight without one module dragging or fighting.
- [ ] Re-enable field-relative driving.
- [ ] Zero the heading, face the robot away from the driver, and confirm field-forward remains
      field-forward.
- [ ] Rotate the robot 90, 180, and 270 degrees and repeat the field-relative check.
- [ ] Verify red-alliance translation reversal behaves as intended.

## 9. Validate odometry and geometry

- [ ] Mark a straight 3.0-meter distance on the floor.
- [ ] Reset odometry, push or drive exactly 3.0 meters, and compare the reported pose distance.
- [ ] Correct wheel diameter/reduction if the error is proportional to distance.
- [ ] Rotate exactly one full turn and confirm heading changes by approximately 360 degrees.
- [ ] Drive a square and confirm the estimated pose returns near its starting point.
- [ ] If rotation produces translation drift, remeasure module spacing and verify module ordering.

## 10. Align PathPlanner physics before autonomous testing

Update `src/main/deploy/pathplanner/settings.json` to match the verified robot:

- [ ] `driveWheelRadius`: 0.0381 m for a true 3-inch wheel, or half the measured loaded diameter.
- [ ] `driveGearing`: 5.08 unless the actual module ratio differs.
- [ ] `maxDriveSpeed`: initially 1.3716 m/s to match the safe Java limit.
- [ ] `driveMotorType`: NEO.
- [ ] `driveCurrentLimit`: 40 A.
- [ ] `flModuleX` and `flModuleY`: +0.2921 m for 23-inch spacing.
- [ ] Use the appropriate signs for the other three module positions.
- [ ] Measure and enter the real competition robot mass including battery and bumpers.
- [ ] Calculate or characterize the robot moment of inertia instead of copying Team 1108's value.
- [ ] Keep default autonomous speed at or below the Java maximum.
- [ ] Build an autonomous chooser and return its selected command from `getAutonomousCommand()`.

## 11. Tune closed-loop control

- [ ] Plot requested and measured drive velocity for every module.
- [ ] Tune drive feedforward/PID only after wheel diameter and gearing are verified.
- [ ] Plot requested and measured steering angle.
- [ ] Reduce steering P if modules oscillate; increase carefully if they respond too weakly.
- [ ] Verify steering settles quickly at 0, 90, 180, and 270 degrees.
- [ ] Verify PathPlanner pose error at low speed before raising autonomous velocity.
- [ ] Tune PathPlanner translation and rotation gains on your robot; Team 1108's value of 5.0 is
      only a starting point.

## 12. Increase performance gradually

- [ ] Raise the speed cap in steps: 0.5, 1.0, 1.37, 2.0 m/s, then higher only if desired.
- [ ] At every step, check current draw, voltage sag, motor temperature, tracking, and stopping
      distance.
- [ ] Never exceed the measured attainable speed. The current 3-inch/5.08:1 calculation predicts
      an unloaded free speed near 4.46 m/s, not Team 1108's 4.8 m/s.
- [ ] Increase PathPlanner constraints only after teleop and pose tracking remain stable.
- [ ] Once the replacement has passed teleop, odometry, and autonomous testing, remove YAGSL and
      its deploy configuration if the team no longer wants the fallback.
