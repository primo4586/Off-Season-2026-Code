package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeArmSim extends IntakeArm {
    private final IntakeSimulation intakeSimulation;
    private final IntakeArm intakeArm;
    private final AbstractDriveTrainSimulation drivetrain;

    public IntakeArmSim(AbstractDriveTrainSimulation driveTrain) {
        intakeArm = new IntakeArm();
        this.drivetrain = driveTrain;
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Fuel",
                // Specify the drivetrain to which this intake is attached
                driveTrain,
                // Width of the intake
                Millimeters.of(665),
                // The extension length of the intake beyond the robot's frame (when activated)
                Millimeters.of(311.37),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK,
                // The intake can hold up to 100 fuels
                100);
    }

    @Override
    public Command OpenIntake() {
        return Commands.runOnce(() -> {
            intakeSimulation.startIntake();
        }).andThen(intakeArm.OpenIntake());
    }

    @Override
    public Command CloseIntake() {
        return Commands.runOnce(() -> {
            intakeSimulation.stopIntake();
        }).andThen(intakeArm.CloseIntake());
    }

    public void updateFuelVisualization(Pose3d robotPose) {
        int count = intakeSimulation.getGamePiecesAmount();

        // Example hopper arrangement: up to 10 visible balls.
        int visibleCount = Math.min(count, 10);
        Pose3d[] fuelPoses = new Pose3d[visibleCount];

        for (int i = 0; i < visibleCount; i++) {
            int layer = i / 4;
            int slot = i % 4;

            double x = -0.2 + (slot % 2) * 0.14;
            double y =-0.1 + (slot / 2) * 0.14;
            double z = 0.2 + layer * 0.13;

            Transform3d offset = new Transform3d(
                    new Translation3d(x, y, z),
                    new Rotation3d());

            fuelPoses[i] = new Pose3d().transformBy(offset);
        }

        // Convert robot-relative positions to field-relative positions.
        Pose3d[] fieldPoses = new Pose3d[visibleCount];

        for (int i = 0; i < visibleCount; i++) {
            fieldPoses[i] = robotPose.transformBy(
                    new Transform3d(
                            fuelPoses[i].getTranslation(),
                            fuelPoses[i].getRotation()));
        }

        Logger.recordOutput("Intake/FuelPoses", fieldPoses);
        Logger.recordOutput("Intake/FuelCount", count);
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        Pose3d robotPose = new Pose3d(
                drivetrain.getSimulatedDriveTrainPose());
        updateFuelVisualization(robotPose);
    }
}
