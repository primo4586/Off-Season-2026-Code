package frc;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
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
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.simulation.RobotBumpSim;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.REST_SPEED;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
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
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SimCommandGroupFactory;

public class RobotSubsystemsManager {
        public final int id;
        public final Hood hood;
        public final IntakeArmSim intakeArm;
        public final Shooter shooter;
        public final IntakeRoller intakeRoller;
        public final Feeder feeder;
        public final Drive drive;
        public final Vision vision;
        public final CommandGroupFactory commandGroupFactory;
        public SimCommandGroupFactory simCommandGroupFactory;
        private final PrimoCalc primoCalc;
        public final RobotBumpSim robotBumpSim = new RobotBumpSim(Drive.getModuleTranslations());;
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController m_driverController;

        // SmartDashboard Calibration
        private final LoggedDashboardChooser<Command> autoChooser;
        public SwerveDriveSimulation driveSimulation = null;
        private final Alliance alliance;
        private final DoubleSupplier cameraRotx = () -> SmartDashboard.getNumber("cameraRotx", 0);
        private final DoubleSupplier cameraRoty = () -> SmartDashboard.getNumber("cameraRoty", 0);

        public RobotSubsystemsManager(int id, Alliance alliance) {
                this.id = id;
                this.alliance = alliance;
                hood = new Hood(id, "Hood" + String.valueOf(id));
                shooter = new Shooter(id, "Shooter" + String.valueOf(id));
                intakeRoller = new IntakeRoller(id, "IntakeRoller" + String.valueOf(id));
                feeder = new Feeder(id, "Feeder" + String.valueOf(id));

                m_driverController = new CommandXboxController(
                                id);

                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig,
                                Constants.ROBOTS_STARTING_POSITIONS[id]);
                drive = new Drive(
                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                new ModuleIOSim(driveSimulation.getModules()[3]),
                                driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                                drive,
                                new VisionIOPhotonVisionSim(
                                                VisionConstants.camera0Name,
                                                VisionConstants.robotToCamera0,
                                                driveSimulation::getSimulatedDriveTrainPose));
                System.out.println(id);
                intakeArm = new IntakeArmSim(driveSimulation, id, "IntakeArm" + String.valueOf(id));

                primoCalc = new PrimoCalc(drive, alliance);
                hood.setDefaultCommand(hood.set(0));
                shooter.setDefaultCommand(shooter.run(REST_SPEED));
                intakeRoller.setDefaultCommand(intakeRoller.set(0));
                commandGroupFactory = new CommandGroupFactory(drive, feeder, hood, intakeArm, intakeRoller, shooter,
                                () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(), primoCalc,cameraRotx,cameraRoty,driveSimulation);
                simCommandGroupFactory = null;

                simCommandGroupFactory = new SimCommandGroupFactory(driveSimulation, commandGroupFactory, intakeArm,
                                primoCalc,id);
                NamedCommands.registerCommand("shoot", simCommandGroupFactory.shootSim());
                NamedCommands.registerCommand("shoot with timeout",
                                simCommandGroupFactory.shootSim().withTimeout(Constants.SHOOT_TIMEOUT)
                                                .andThen(intakeArm.OpenIntake()));
                m_driverController.rightTrigger().whileTrue(simCommandGroupFactory.shootSim());
                // named commands

                NamedCommands.registerCommand("open intake", intakeArm.OpenIntake());
                NamedCommands.registerCommand("intake", intakeRoller.intake());
                NamedCommands.registerCommand("stop all", commandGroupFactory.stopAll());
                autoChooser = new LoggedDashboardChooser<>("Auto Choices" + String.valueOf(id),
                                AutoBuilder.buildAutoChooser());
                configureBindings();
        }

        private void configureBindings() {
                final Supplier<Rotation2d> driveRotationOffset;

                driveRotationOffset = () -> new Rotation2d(cameraRotx.getAsDouble(), cameraRoty.getAsDouble());
                drive.setDefaultCommand(DriveCommands.cameraRelativeDrive(
        drive,
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        driveRotationOffset,
        () -> driveSimulation.getSimulatedDriveTrainPose().getRotation()));
                final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                                : () -> drive.resetOdometry(
                                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

                m_driverController.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

                // intake roller test
                m_driverController.x().whileTrue(intakeRoller.intakeWithVoltage());
                m_driverController.b().whileTrue(intakeRoller.outakeWithVoltage());
                m_driverController.povUp().onTrue(intakeArm.OpenIntake());
                m_driverController.povDown().onTrue(intakeArm.CloseIntake());
                m_driverController.povRight().whileTrue(hood.resetHood());
                m_driverController.a().whileTrue(hood.run(Degrees.of(15)));
                m_driverController.y().whileTrue(simCommandGroupFactory.shootSim());

        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public void resetHood() {
                hood.setEncoderPostision(0);
                ;
        }
}
