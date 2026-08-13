package frc.robot.drive1108;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** One NEO/NEO 550/SPARK MAX/Through Bore Encoder MAXSwerve module. */
public class MAXSwerveModule {
  private final SparkMax drivingSpark;
  private final SparkMax turningSpark;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;
  private final SparkClosedLoopController drivingController;
  private final SparkClosedLoopController turningController;
  private final double chassisAngularOffset;

  public MAXSwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingSpark = new SparkMax(drivingCanId, MotorType.kBrushless);
    turningSpark = new SparkMax(turningCanId, MotorType.kBrushless);
    drivingEncoder = drivingSpark.getEncoder();
    turningEncoder = turningSpark.getAbsoluteEncoder();
    drivingController = drivingSpark.getClosedLoopController();
    turningController = turningSpark.getClosedLoopController();
    this.chassisAngularOffset = chassisAngularOffset;

    drivingSpark.configure(
        DriveConfigs.DRIVE, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningSpark.configure(
        DriveConfigs.TURN, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    drivingEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedState =
        new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset)));

    correctedState.optimize(new Rotation2d(turningEncoder.getPosition()));
    drivingController.setSetpoint(correctedState.speedMetersPerSecond, ControlType.kVelocity);
    turningController.setSetpoint(correctedState.angle.getRadians(), ControlType.kPosition);
  }

  public void resetDriveEncoder() {
    drivingEncoder.setPosition(0.0);
  }

  public double getAbsoluteAngleRadians() {
    return turningEncoder.getPosition();
  }
}
