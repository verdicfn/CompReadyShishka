package frc.robot.drive1108;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Hardware constants for the Team 1108-style drive implementation. */
public final class DriveConstants {
  private DriveConstants() {}

  // Verified from this robot's existing YAGSL configuration.
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(23.0);
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(23.0);

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
          new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
          new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
          new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0));

  public static final int FRONT_LEFT_DRIVE_CAN_ID = 7;
  public static final int FRONT_LEFT_TURN_CAN_ID = 8;
  public static final int FRONT_RIGHT_DRIVE_CAN_ID = 5;
  public static final int FRONT_RIGHT_TURN_CAN_ID = 6;
  public static final int REAR_LEFT_DRIVE_CAN_ID = 1;
  public static final int REAR_LEFT_TURN_CAN_ID = 2;
  public static final int REAR_RIGHT_DRIVE_CAN_ID = 3;
  public static final int REAR_RIGHT_TURN_CAN_ID = 4;
  public static final int PIGEON_CAN_ID = 9;

  // Existing YAGSL offsets are all zero. Do not activate this drive until that calibration is
  // confirmed using the bring-up checklist.
  public static final double FRONT_LEFT_ANGULAR_OFFSET_RADIANS = 0.0;
  public static final double FRONT_RIGHT_ANGULAR_OFFSET_RADIANS = 0.0;
  public static final double REAR_LEFT_ANGULAR_OFFSET_RADIANS = 0.0;
  public static final double REAR_RIGHT_ANGULAR_OFFSET_RADIANS = 0.0;

  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.0);
  public static final double DRIVE_REDUCTION = 5.08;
  public static final double NEO_FREE_SPEED_RPM = 5676.0;
  public static final double DRIVE_POSITION_FACTOR =
      WHEEL_DIAMETER_METERS * Math.PI / DRIVE_REDUCTION;
  public static final double THEORETICAL_FREE_SPEED_METERS_PER_SECOND =
      (NEO_FREE_SPEED_RPM / 60.0) * DRIVE_POSITION_FACTOR;

  // Start at the old drivebase's 4.5 ft/s cap. Raise only after completing the checklist.
  public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(4.5);
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;

  public static final int DRIVE_CURRENT_LIMIT_AMPS = 40;
  public static final int TURN_CURRENT_LIMIT_AMPS = 20;
  public static final double DRIVE_P = 0.04;
  public static final double TURN_P = 1.0;
  public static final double PATH_TRANSLATION_P = 5.0;
  public static final double PATH_ROTATION_P = 5.0;
}
