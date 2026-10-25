// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;

import frc.robot.subsystems.IntakeArm.IntakeArm;
import frc.robot.subsystems.IntakeArm.IntakeArmSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.simulation.RobotBumpSim;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.subsystems.shooter.ShooterConstants.REST_SPEED;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SimCommandGroupFactory;

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
    // private final Feeder feeder = new Feeder();
    private final Hood hood = new Hood();
    private final IntakeArm intakeArm;
    private final Shooter shooter = new Shooter();
    private final IntakeRoller intakeRoller = new IntakeRoller();
    private final Feeder feeder = new Feeder();
    private final Drive drive;
    private final Vision vision;
    private final CommandGroupFactory commandGroupFactory;
    private SimCommandGroupFactory simCommandGroupFactory;
    RobotBumpSim robotBumpSim = new RobotBumpSim(Drive.getModuleTranslations());;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    // SmartDashboard Calibration
    private final DoubleSupplier shooterRPM = () -> SmartDashboard.getNumber("shooter calibration RPM", 0.0);
    private final DoubleSupplier hoodAngle = () -> SmartDashboard.getNumber("hood angle rotations", 0.0);
    private final LoggedDashboardChooser<Command> autoChooser;
    private double hoodpos = 0;
    private SwerveDriveSimulation driveSimulation = null;
    IntakeArmSim intakeArmSim = null;

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
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (robotPose) -> {
                        });
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));
                intakeArm = new IntakeArm();
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
                arena.setEfficiencyMode(false);
                SimulatedArena.overrideInstance(arena);
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = Drive.getInstance(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name, VisionConstants.robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose));
                intakeArmSim = new IntakeArmSim(driveSimulation);
                intakeArm = intakeArmSim;
                break;

            default:
                // Replayed robot, disable IO implementations0
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        (robotPose) -> {
                        });
                vision = new Vision(drive, new VisionIO() {
                }, new VisionIO() {
                });
                intakeArm = new IntakeArm();
                break;
        }
        hood.setDefaultCommand(hood.set(0));
        shooter.setDefaultCommand(shooter.run(REST_SPEED));
        intakeRoller.setDefaultCommand(intakeRoller.set(0));
        commandGroupFactory = new CommandGroupFactory(drive, feeder, hood, intakeArm, intakeRoller, shooter,
                () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX());
        simCommandGroupFactory = null;

        if (Constants.currentMode == Mode.SIM) {
            simCommandGroupFactory = new SimCommandGroupFactory(drive, commandGroupFactory, intakeArmSim);
            NamedCommands.registerCommand("shoot", simCommandGroupFactory.shootSim());
            NamedCommands.registerCommand("shoot with timeout",
                    simCommandGroupFactory.shootSim().withTimeout(Constants.SHOOT_TIMEOUT).andThen(intakeArm.OpenIntake()));
            m_driverController.rightTrigger().whileTrue(simCommandGroupFactory.shootSim());
        } else {
            NamedCommands.registerCommand("shoot", commandGroupFactory.shoot());
            NamedCommands.registerCommand("shoot with timeout",
                    commandGroupFactory.shoot().withTimeout(Constants.SHOOT_TIMEOUT));
        }
        // named commands

        NamedCommands.registerCommand("open intake", intakeArm.OpenIntake());
        NamedCommands.registerCommand("intake", intakeRoller.intake());
        NamedCommands.registerCommand("stop all", commandGroupFactory.stopAll());
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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

        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()));
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        m_driverController.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

        // intake roller test
        m_driverController.x().whileTrue(intakeRoller.intakeWithVoltage());
        m_driverController.b().whileTrue(intakeRoller.outakeWithVoltage());
        m_driverController.povUp().onTrue(intakeArm.OpenIntake());
        m_driverController.povDown().onTrue(intakeArm.CloseIntake());
        m_driverController.povRight().whileTrue(hood.resetHood());
        m_driverController.a().whileTrue(hood.run(Degrees.of(15)));
        m_driverController.y().whileTrue(simCommandGroupFactory.shootSim());
        /*
         * // shooter test
         * m_driverController.a().whileTrue(shooter.run(RPM.of(shooterRPM.getAsDouble())
         * ));
         */

        // shooter test
        m_driverController.a().whileTrue(shooter.run(RPM.of(shooterRPM.getAsDouble())));

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
        SmartDashboard.putNumber("Distance From Hub", PrimoCalc.getDistance(drive.getPose(), PrimoCalc.getHubPos()));
        SmartDashboard.putNumber("interpolted HOOD value", PrimoCalc.interplateHood().get().in(Degrees));
        SmartDashboard.putNumber("interpolted SHOOTER value", PrimoCalc.interplateShooter().get().in(RPM));
        SmartDashboard.putBoolean("in shooting distance",
                PrimoCalc.getDistance(drive.getPose(), PrimoCalc.getHubPos()) < 4.6 &&
                        PrimoCalc.getDistance(drive.getPose(), PrimoCalc.getHubPos()) > 1.39);
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
        if (intakeArmSim != null) {
            intakeArmSim.setFuel(8);
        }
        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null)
            return;

        // Step 1: Step the 2D world
        SimulatedArena.getInstance().simulationPeriodic();

        // Step 2: Get true simulation pose and field-relative chassis speeds
        Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();
        ChassisSpeeds fieldSpeeds = driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

        // Step 3: Pass fieldSpeeds into RobotBumpSim
        Pose3d simPose3d = robotBumpSim.update(simPose, fieldSpeeds, 5);
        if (robotBumpSim.isOnRamp()) {
            driveSimulation.setSimulationWorldPose(robotBumpSim.getSimWorldPose(simPose));
        } else {
            // Fallback 3D pose when on flat ground (Z = 0)
            simPose3d = new Pose3d(simPose);
        }

        Logger.recordOutput("Drive/Pose3d", simPose3d);
        Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
        Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
    }
    public void resetIntake(){
        intakeArm.CloseIntake();
    }

}
