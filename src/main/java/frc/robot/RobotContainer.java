// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.RobotSubsystemsManager;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

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
    RobotSubsystemsManager[] robots = new RobotSubsystemsManager[6];

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        SmartDashboard.putNumber("cameraRotx", 0);
        SmartDashboard.putNumber("cameraRoty", -180);
        if (Constants.currentMode == Constants.Mode.SIM) {
            Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
            arena.setEfficiencyMode(false);
            Logger.recordOutput("Debug/RedShootPoses", RebuiltHub.redShootPoses);
            // Workaround for RebuiltHub.redShootPoses bug (poses end up off-field / wrong
            // heading).
            // Rebuilt from the blue chute layout, mirrored across the field's centerline.
            double[] yOffsets = { 0.447675, 0.149225, -0.149225, -0.447675 };
            double[] blueYawDeg = { 33.75, 11.25, -11.25, -33.75 }; // symmetric spread
            for (int i = 0; i < RebuiltHub.redShootPoses.length; i++) {
                RebuiltHub.redShootPoses[i] = new Pose3d(
                        new Translation3d(11.938 - 0.5969, 4.034536 + yOffsets[i], 1.5748 - 0.5),
                        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180 - blueYawDeg[i])));
            }
            SimulatedArena.overrideInstance(arena);
        }
        for (int i = 0; i < robots.length; i++) {
            robots[i] = new RobotSubsystemsManager(i, Constants.idToAliance[i]);
            SimulatedArena.getInstance().addDriveTrainSimulation(robots[i].driveSimulation);
        }

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

    /**
     * Use this to pass the autonomous command t o the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public void periodic() {
        Logger.recordOutput("score/redscore", SimulatedArena.getInstance().getScore(false));
        Logger.recordOutput("score/bluescore", SimulatedArena.getInstance().getScore(true));

        for (RobotSubsystemsManager robot : robots) {
            robot.periodic();
        }
    }

    public void stopAll() {
        for (RobotSubsystemsManager robot : robots) {
            CommandScheduler.getInstance().schedule(robot.commandGroupFactory.stopAll());
            ;
        }
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
        for (RobotSubsystemsManager robot : robots) {
            if (robot.intakeArm != null) {
                robot.intakeArm.setFuel(8);
            }
        }

        for (RobotSubsystemsManager robot : robots) {
            robot.drive.resetOdometry(Constants.ROBOTS_STARTING_POSITIONS[robot.id]);
        }
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        // Step 1: Step the 2D world
        SimulatedArena.getInstance().simulationPeriodic();
        for (RobotSubsystemsManager robot : robots) {
            Pose2d simPose = robot.driveSimulation.getSimulatedDriveTrainPose();
            ChassisSpeeds fieldSpeeds = robot.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

            // Step 3: Pass fieldSpeeds into RobotBumpSim
            Pose3d simPose3d = robot.robotBumpSim.update(simPose, fieldSpeeds, 5);
            if (robot.robotBumpSim.isOnRamp()) {
                robot.driveSimulation.setSimulationWorldPose(robot.robotBumpSim.getSimWorldPose(simPose));
            } else {
                // Fallback 3D pose when on flat ground (Z = 0)
                simPose3d = new Pose3d(simPose);
            }

            Logger.recordOutput("Robots/Robot" + robot.id, simPose3d);
        }
        // Step 2: Get true simulation pose and field-relative chassis speeds

        Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
        Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
    }

    public void resetIntake() {
        for (RobotSubsystemsManager robot : robots) {
            robot.intakeArm.CloseIntake();
        }
    }

}
