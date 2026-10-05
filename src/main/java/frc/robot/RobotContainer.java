// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;

import frc.robot.subsystems.IntakeArm.IntakeArm;
import frc.robot.subsystems.shooter.Shooter;
import yams.mechanisms.swerve.utility.SwerveInputStream;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.subsystems.shooter.ShooterConstants.REST_SPEED;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Feeder feeder = new Feeder();
  private final Hood hood = new Hood();
  // private final IntakeArm intakeArm = new IntakeArm();
  private final Shooter shooter = new Shooter();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final Drive drive = new Drive();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
      private       boolean headingControlEnabled = false;
  private final SwerveInputStream driveStream = drive.getAngularVelocityStream(m_driverController::getLeftY,
                                                                                 m_driverController::getLeftX,
                                                                                 ()->m_driverController.getRawAxis(2))
                                                       .withControllerHeadingAxis(m_driverController::getRightX,
                                                                                  m_driverController::getRightY)
                                                       .withHeadingControl(() -> headingControlEnabled)
                                                       .withDeadband(0.05)
                                                       .withAllianceRelativeControl();
  // SmartDashboard Calibration
  private final DoubleSupplier shooterRPM = () -> SmartDashboard.getNumber("shooter calibration RPM", 0.0);
  private final DoubleSupplier hoodAngle = () -> SmartDashboard.getNumber("hood angle rotations", 0.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putNumber("shooter calibration RPM", 100);
    SmartDashboard.putNumber("hood angle rotations", 1);
    configureBindings();
    feeder.setDefaultCommand(feeder.set(0));
    hood.setDefaultCommand(hood.set(0));
    shooter.setDefaultCommand(shooter.run(REST_SPEED));
    intakeRoller.setDefaultCommand(intakeRoller.set(0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(drive.drive(driveStream));
    /*
     * // intake arm test
     * m_driverController.a().whileTrue(intakeArm.openWithVoltage());
     * m_driverController.b().whileTrue(intakeArm.closeWithVoltage());
     */
    /*
     * // intake roller test
     * m_driverController.a().whileTrue(intakeRoller.intakeWithVoltage());
     * m_driverController.b().whileTrue(intakeRoller.outakeWithVoltage());
     */
    /*
     * // hood test
     * m_driverController.a().whileTrue(hood.resetHood());
     * m_driverController.b().whileTrue(hood.run(Rotation.of(hoodAngle.getAsDouble()
     * )
     * ));
     */
    /*
     * // shooter test
     * m_driverController.a().whileTrue(shooter.run(RPM.of(shooterRPM.getAsDouble())
     * ));
     */

  }

  public void periodic() {
  }
}
