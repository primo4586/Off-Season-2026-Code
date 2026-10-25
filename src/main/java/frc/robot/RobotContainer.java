// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;

import frc.robot.subsystems.IntakeArm.IntakeArm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.REST_SPEED;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.commands.DriveCommands;

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
    private final IntakeArm intakeArm = new IntakeArm();
    private final Shooter shooter = new Shooter();
    private final IntakeRoller intakeRoller = new IntakeRoller();
    private final Drive drive;
    private final Vision vision;
    private final CommandGroupFactory commandGroupFactory;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    // SmartDashboard Calibration
    private final DoubleSupplier shooterRPM = () -> SmartDashboard.getNumber("shooter calibration RPM", 0.0);
    private final DoubleSupplier hoodAngle = () -> SmartDashboard.getNumber("hood angle rotations", 0.0);
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        SmartDashboard.putNumber("shooter calibration RPM", 100);
        SmartDashboard.putNumber("hood angle rotations", 10);
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = Drive.getInstance(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVision(VisionConstants.fowordCameraName, VisionConstants.fowordTransform));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = Drive.getInstance(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(VisionConstants.fowordCameraName, VisionConstants.fowordTransform,
                                drive::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations0
                drive = Drive.getInstance(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                break;
        }
        // setup defualt commands and commandgroupfactory
        feeder.setDefaultCommand(feeder.set(0));
        hood.setDefaultCommand(hood.set(0));
        shooter.setDefaultCommand(shooter.run(REST_SPEED));
        intakeRoller.setDefaultCommand(intakeRoller.set(0));
        commandGroupFactory = new CommandGroupFactory(drive, feeder, hood, intakeArm, intakeRoller, shooter,
                () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX());
        // setup defualtcommands
        // named commands
        NamedCommands.registerCommand("shoot", commandGroupFactory.shoot());
        NamedCommands.registerCommand("shoot with timeout",
                commandGroupFactory.shoot().withTimeout(Constants.SHOOT_TIMEOUT));
        NamedCommands.registerCommand("open intake", intakeArm.OpenIntake());

        NamedCommands.registerCommand("intake", intakeRoller.intake());
        NamedCommands.registerCommand("stop all", commandGroupFactory.stopAll());
        // setup auto
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        configureBindings();

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

        m_driverController.rightTrigger().whileTrue(commandGroupFactory.shoot());
        m_driverController.rightBumper().whileTrue(commandGroupFactory.closeShoot(RotationsPerSecond.of(57.75)));
        m_driverController.leftTrigger().whileTrue(commandGroupFactory.pass());
        m_driverController.povRight().whileTrue(intakeRoller.intake());
        m_driverController.y().whileTrue(intakeRoller.outake());
        m_driverController.x().whileTrue(intakeArm.CloseIntake());
        m_driverController.b().onTrue(intakeArm.OpenIntake());
        m_driverController.a().whileTrue(hood.resetHood());
        m_driverController.povLeft().whileTrue(feeder.unfeed());
        // normal movment
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -m_driverController.getLeftY(),
                        () -> -m_driverController.getLeftX(),

                        () -> -m_driverController.getRightX() * 0.75)
                        .withName("Drive"));
        m_driverController
                .rightStick()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -m_driverController.getLeftY(),
                                () -> -m_driverController.getLeftX(),
                                () -> new Rotation2d(
                                        Math.toRadians(PrimoCalc.bumpAngle(drive.getRotation().getDegrees()))))
                                .withName("Drive At Angle"));
        // Drive towards ds wall
        m_driverController
                .povDown()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -m_driverController.getLeftY(),
                                () -> -m_driverController.getLeftX(),
                                () -> PrimoCalc.getRotToDsWall())
                                .withName("Drive At Angle"));
        // gyro reset
        m_driverController
                .povUp()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                                drive)
                                .ignoringDisable(true));

    }

    /**
     * Use this to pass the autonomous command t o the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetHood() {
        hood.setEncoderPostision(0);
        ;
    }

    public void periodic() {
    }
}
