package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.IntakeArm.IntakeArmSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class SimCommandGroupFactory {
        private final CommandGroupFactory commandGroupFactory;
        private final SimulatedArena simulatedArena = SimulatedArena.getInstance();
        private Pose2d pose;
        private final SwerveDriveSimulation drivesim;
        private final IntakeArmSim intakeSimulation;
        private final PrimoCalc primoCalc;
        private final int id;
        public SimCommandGroupFactory(SwerveDriveSimulation drive, CommandGroupFactory commandGroupFactory,
                        IntakeArmSim intakeSimulation, PrimoCalc primoCalc, int id) {
                this.id= id;
                this.commandGroupFactory = commandGroupFactory;
                this.drivesim = drive;
                this.intakeSimulation = intakeSimulation;
                this.primoCalc = primoCalc;
        }

        public void shootSimulatedFuel() {
                Pose2d pose = drivesim.getSimulatedDriveTrainPose();
                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                                // Specify the position of the chassis when the fuel is launched
                                pose.getTranslation(),
                                // Specify the translation of the shooter from the robot center (in the
                                // shooter’s reference frame)
                                new Translation2d(0,
                                                0),
                                // Specify the field-relative speed of the chassis, adding it to the initial
                                // velocity of the projectile
                                drivesim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                // The shooter facing direction is the same as the robot’s facing direction
                                pose.getRotation(),
                                // Initial height of the flying fuel
                                Millimeters.of(900),
                                // The launch speed is proportional to the RPM; assumed to be 16 meters/second
                                // at 6000 RPM
                                MetersPerSecond.of(6.75),
                                // The angle at which the fuel is launched
                                Degrees.of(45)

                );
                intakeSimulation.removeFuel(1);
                simulatedArena.addGamePieceProjectile(fuelOnFly);
        }

        public Command shootSim() {
                return Commands.parallel(
                                commandGroupFactory.prepareToShoot(true),
                                Commands.run(
                                                () -> Logger.recordOutput("readyToShoot"+String.valueOf(id),
                                                                commandGroupFactory.isReadyToShoot(true))),
                                Commands.run(() -> {
                                        if (commandGroupFactory.isReadyToShoot(true).getAsBoolean()
                                                        && intakeSimulation.getFuelCount() >= 1) {
                                                shootSimulatedFuel();
                                        }
                                }));
        }

}
