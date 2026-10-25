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

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
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
        private final Map<String, Supplier<Command>> namedCommands = new HashMap<>();

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
                                driveSimulation::setSimulationWorldPose, alliance, id);

                vision = new Vision(
                                drive,
                                new VisionIOPhotonVisionSim(
                                                VisionConstants.camera0Name,
                                                VisionConstants.robotToCamera0,
                                                driveSimulation::getSimulatedDriveTrainPose));
                intakeArm = new IntakeArmSim(driveSimulation, id, "IntakeArm" + String.valueOf(id));

                primoCalc = new PrimoCalc(drive, alliance);
                hood.setDefaultCommand(hood.set(0));
                shooter.setDefaultCommand(shooter.run(REST_SPEED));
                intakeRoller.setDefaultCommand(intakeRoller.set(0));
                commandGroupFactory = new CommandGroupFactory(drive, feeder, hood, intakeArm, intakeRoller, shooter,
                                () -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(), primoCalc,
                                cameraRotx, cameraRoty, driveSimulation);
                simCommandGroupFactory = null;

                simCommandGroupFactory = new SimCommandGroupFactory(driveSimulation, commandGroupFactory, intakeArm,
                                primoCalc, id);

                m_driverController.rightTrigger().whileTrue(simCommandGroupFactory.shootSim());

                // named commands
                namedCommands.put("shoot", () -> simCommandGroupFactory.shootSim());
                namedCommands.put("shoot with timeout", () -> simCommandGroupFactory.shootSim()
                                .withTimeout(Constants.SHOOT_TIMEOUT)
                                .andThen(intakeArm.OpenIntake()));
                namedCommands.put("open intake", () -> intakeArm.OpenIntake());
                namedCommands.put("intake", () -> intakeRoller.intake());
                namedCommands.put("stop all", () -> commandGroupFactory.stopAll());
                autoChooser = new LoggedDashboardChooser<>("Auto Choices" + String.valueOf(id));
                for (String name : AutoBuilder.getAllAutoNames()) {
                        autoChooser.addOption(name, buildAuto(name));
                }
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

        private Command buildAuto(String autoName) {
                try {
                        File file = new File(Filesystem.getDeployDirectory(),
                                        "pathplanner/autos/" + autoName + ".auto");
                        JsonNode root = new ObjectMapper().readTree(file);
                        Command body = parseCommand(root.path("command"));

                        // Reset odometry to the start of the first path (flipped for red)
                        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                        if (!paths.isEmpty()) {
                                PathPlannerPath first = alliance == Alliance.Red ? paths.get(0).flipPath()
                                                : paths.get(0);
                                var start = first.getStartingHolonomicPose();
                                if (start.isPresent()) {
                                        body = Commands.runOnce(() -> drive.resetOdometry(start.get())).andThen(body);
                                }
                        }
                        return body.withName(autoName);
                } catch (Exception e) {
                        DriverStation.reportError("Failed to build auto " + autoName + ": " + e.getMessage(),
                                        e.getStackTrace());
                        return Commands.none();
                }
        }

        private Command parseCommand(JsonNode node) throws Exception {
                JsonNode data = node.path("data");
                switch (node.path("type").asText()) {
                        case "path":
                                return followPath(data.path("pathName").asText());
                        case "named": {
                                String name = data.path("name").asText();
                                Supplier<Command> s = namedCommands.get(name);
                                if (s == null) {
                                        DriverStation.reportWarning("Unknown named command: " + name, false);
                                        return Commands.none();
                                }
                                return s.get();
                        }
                        case "wait":
                                return Commands.waitSeconds(data.path("waitTime").asDouble());
                        case "sequential":
                                return Commands.sequence(parseList(data.path("commands")));
                        case "parallel":
                                return Commands.parallel(parseList(data.path("commands")));
                        case "race":
                                return Commands.race(parseList(data.path("commands")));
                        case "deadline": {
                                Command[] cmds = parseList(data.path("commands"));
                                return Commands.deadline(cmds[0], Arrays.copyOfRange(cmds, 1, cmds.length));
                        }
                        default:
                                DriverStation.reportWarning(
                                                "Unsupported auto command type: " + node.path("type").asText(), false);
                                return Commands.none();
                }
        }

        private Command[] parseList(JsonNode array) throws Exception {
                List<Command> list = new ArrayList<>();
                for (JsonNode child : array) {
                        list.add(parseCommand(child));
                }
                return list.toArray(new Command[0]);
        }

        private Command followPath(String pathName) throws Exception {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                if (alliance == Alliance.Red) {
                        path = path.flipPath();
                }
                return new FollowPathCommand(
                                path,
                                drive::getPose,
                                drive::getChassisSpeeds,
                                (speeds, ff) -> drive.runVelocity(speeds),
                                new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
                                Drive.PP_CONFIG,
                                () -> false, // already flipped above
                                drive);
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public void resetHood() {
                hood.setEncoderPostision(0);
                ;
        }

        public void periodic() {
                Logger.recordOutput("SimTruth/Robot" + id, driveSimulation.getSimulatedDriveTrainPose());
        }
}
