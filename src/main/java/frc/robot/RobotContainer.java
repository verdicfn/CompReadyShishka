// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.drive1108.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final DriveSubsystem drivebase = new DriveSubsystem();
    private final SendableChooser<Command> autonomousChooser = AutoBuilder.buildAutoChooser();
    private boolean invertDriveBindings = false;
  
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      configureBindings();
      drivebase.setDefaultCommand(drivebase.run(() -> drivebase.drive(
          applyDriveInversion(-MathUtil.applyDeadband(
              m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)),
          applyDriveInversion(-MathUtil.applyDeadband(
              m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)),
          applyDriveInversion(-MathUtil.applyDeadband(
              m_driverController.getRightX(), OperatorConstants.LEFT_X_DEADBAND)),
          true)));
      SmartDashboard.putData("Autonomous", autonomousChooser);
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Toggle manual drive inversion on Y for alliance-side perspective changes.
    m_driverController.y().onTrue(Commands.runOnce(() -> {
      invertDriveBindings = !invertDriveBindings;
      SmartDashboard.putBoolean("Drive Controls Inverted", invertDriveBindings);
    }));

    m_driverController.start().onTrue(Commands.runOnce(drivebase::zeroHeading, drivebase));
    m_driverController.x().whileTrue(drivebase.run(drivebase::setX));

    SmartDashboard.putBoolean("Drive Controls Inverted", invertDriveBindings);
  }

  private double applyDriveInversion(double value) {
    return invertDriveBindings ? -value : value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
