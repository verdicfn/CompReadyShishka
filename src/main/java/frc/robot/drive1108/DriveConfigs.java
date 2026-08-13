package frc.robot.drive1108;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** SPARK MAX configurations following Team 1108's MAXSwerve implementation. */
public final class DriveConfigs {
  private DriveConfigs() {}

  public static final SparkMaxConfig DRIVE = new SparkMaxConfig();
  public static final SparkMaxConfig TURN = new SparkMaxConfig();

  static {
    double velocityFactor = DriveConstants.DRIVE_POSITION_FACTOR / 60.0;
    double velocityFeedForward =
        12.0 / DriveConstants.THEORETICAL_FREE_SPEED_METERS_PER_SECOND;

    DRIVE
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.DRIVE_CURRENT_LIMIT_AMPS);
    DRIVE.encoder
        .positionConversionFactor(DriveConstants.DRIVE_POSITION_FACTOR)
        .velocityConversionFactor(velocityFactor);
    DRIVE.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DriveConstants.DRIVE_P, 0.0, 0.0)
        .outputRange(-1.0, 1.0)
        .feedForward.kV(velocityFeedForward);

    TURN
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.TURN_CURRENT_LIMIT_AMPS);
    TURN.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor(2.0 * Math.PI / 60.0)
        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
    TURN.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(DriveConstants.TURN_P, 0.0, 0.0)
        .outputRange(-1.0, 1.0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 2.0 * Math.PI);
  }
}
