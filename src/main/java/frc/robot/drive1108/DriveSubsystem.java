package frc.robot.drive1108;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Team 1108's direct MAXSwerve drive architecture, adapted to this robot's CAN IDs and Pigeon 2.
 * This class is intentionally not instantiated until the hardware bring-up checklist is complete.
 */
public class DriveSubsystem extends SubsystemBase {
  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          DriveConstants.FRONT_LEFT_DRIVE_CAN_ID,
          DriveConstants.FRONT_LEFT_TURN_CAN_ID,
          DriveConstants.FRONT_LEFT_ANGULAR_OFFSET_RADIANS);
  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_CAN_ID,
          DriveConstants.FRONT_RIGHT_TURN_CAN_ID,
          DriveConstants.FRONT_RIGHT_ANGULAR_OFFSET_RADIANS);
  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          DriveConstants.REAR_LEFT_DRIVE_CAN_ID,
          DriveConstants.REAR_LEFT_TURN_CAN_ID,
          DriveConstants.REAR_LEFT_ANGULAR_OFFSET_RADIANS);
  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          DriveConstants.REAR_RIGHT_DRIVE_CAN_ID,
          DriveConstants.REAR_RIGHT_TURN_CAN_ID,
          DriveConstants.REAR_RIGHT_ANGULAR_OFFSET_RADIANS);

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.PIGEON_CAN_ID);
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          gyro.getRotation2d(),
          getModulePositions(),
          new Pose2d());

  public DriveSubsystem() {
    configurePathPlanner();
  }

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation2d(), getModulePositions());
  }

  /** Accepts normalized driver inputs from -1 to 1. */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotationDelivered = rotation * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    if (fieldRelative
        && DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red) {
      xSpeedDelivered = -xSpeedDelivered;
      ySpeedDelivered = -ySpeedDelivered;
    }

    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotationDelivered,
                gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered);
    driveRobotRelative(speeds);
  }

  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.KINEMATICS.toChassisSpeeds(
        frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(speeds));
  }

  public void stop() {
    driveRobotRelative(new ChassisSpeeds());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public void resetDriveEncoders() {
    frontLeft.resetDriveEncoder();
    frontRight.resetDriveEncoder();
    rearLeft.resetDriveEncoder();
    rearRight.resetDriveEncoder();
  }

  public void zeroHeading() {
    gyro.setYaw(
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red
            ? 180.0
            : 0.0);
  }

  public double getHeadingDegrees() {
    return gyro.getRotation2d().getDegrees();
  }

  public void addVisionMeasurement(Pose2d estimatedPose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(estimatedPose, timestampSeconds);
  }

  public void addVisionMeasurementWithStandardDeviations(
      Pose2d estimatedPose,
      double timestampSeconds,
      double xyStandardDeviationMeters,
      double rotationStandardDeviationRadians) {
    poseEstimator.addVisionMeasurement(
        estimatedPose,
        timestampSeconds,
        VecBuilder.fill(
            xyStandardDeviationMeters,
            xyStandardDeviationMeters,
            rotationStandardDeviationRadians));
  }

  public double[] getAbsoluteAnglesRadians() {
    return new double[] {
      frontLeft.getAbsoluteAngleRadians(),
      frontRight.getAbsoluteAngleRadians(),
      rearLeft.getAbsoluteAngleRadians(),
      rearRight.getAbsoluteAngleRadians()
    };
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    };
  }

  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              new PIDConstants(DriveConstants.PATH_TRANSLATION_P, 0.0, 0.0),
              new PIDConstants(DriveConstants.PATH_ROTATION_P, 0.0, 0.0)),
          config,
          () ->
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                  == DriverStation.Alliance.Red,
          this);
    } catch (Exception exception) {
      DriverStation.reportError(
          "Failed to configure Team 1108-style PathPlanner drive: "
              + exception.getMessage(),
          exception.getStackTrace());
    }
  }
}
