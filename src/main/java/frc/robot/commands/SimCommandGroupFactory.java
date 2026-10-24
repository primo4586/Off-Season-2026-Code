package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
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
    private final Drive drive;
    private final IntakeArmSim intakeSimulation;
    static int score = 0;

    public SimCommandGroupFactory(Drive drive, CommandGroupFactory commandGroupFactory, IntakeArmSim intakeSimulation) {
        this.commandGroupFactory = commandGroupFactory;
        this.drive = drive;
        this.intakeSimulation = intakeSimulation;
        SmartDashboard.putNumber("shootersim/startingx", 0);
        SmartDashboard.putNumber("shootersim/startingy", 0);
        SmartDashboard.putNumber("shootersim/startingz", 900);
        SmartDashboard.putNumber("shootersim/startingspeed", 6.75);
        SmartDashboard.putNumber("shootersim/startingdegress", 45);

    }

    public void shootSimulatedFuel() {
        Logger.recordOutput("Shooter/RobotX", drive.getPose().getX());
        Logger.recordOutput("Shooter/RobotY", drive.getPose().getY());
        Logger.recordOutput("Shooter/RobotHeadingDeg", drive.getRotation().getDegrees());

        Logger.recordOutput("Shooter/HubX",
                commandGroupFactory.primoCalc.getHubPos().getX());
        Logger.recordOutput("Shooter/HubY",
                commandGroupFactory.primoCalc.getHubPos().getY());

        Logger.recordOutput(
                "Shooter/DistanceToHub",
                drive.getPose().getTranslation().getDistance(
                        commandGroupFactory.primoCalc.getHubPos().toPose2d().getTranslation()));
        RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                // Specify the position of the chassis when the fuel is launched
                drive.getPose().getTranslation(),
                // Specify the translation of the shooter from the robot center (in the
                // shooter’s reference frame)
                new Translation2d(SmartDashboard.getNumber("shootersim/startingx", 0),
                        SmartDashboard.getNumber("shootersim/startingy", 0)),
                // Specify the field-relative speed of the chassis, adding it to the initial
                // velocity of the projectile
                drive.getChassisSpeeds(),
                // The shooter facing direction is the same as the robot’s facing direction
                drive.getRotation(),
                // Initial height of the flying fuel
                Millimeters.of(SmartDashboard.getNumber("shootersim/startingz", 0)),
                // The launch speed is proportional to the RPM; assumed to be 16 meters/second
                // at 6000 RPM
                MetersPerSecond.of(SmartDashboard.getNumber("shootersim/startingspeed", 0)),
                // The angle at which the fuel is launched
                Degrees.of(SmartDashboard.getNumber("shootersim/startingdegress", 0))

        );
        fuelOnFly
                .withProjectileTrajectoryDisplayCallBack(
                        // Callback for when the fuel will eventually hit the target (if configured)
                        (pose3ds) -> Logger.recordOutput("Shooter/FuelProjectileSuccessfulShot",
                                pose3ds.toArray(Pose3d[]::new)),
                        // Callback for when the fuel will eventually miss the target, or if no target
                        // is configured
                        (pose3ds) -> Logger.recordOutput("Shooter/FuelProjectileUnsuccessfulShot",
                                pose3ds.toArray(Pose3d[]::new)))
                .withTargetPosition(() -> commandGroupFactory.primoCalc.getHubPos().getTranslation())
                .withHitTargetCallBack(() -> {
                    score += 1;
                    Logger.recordOutput("Shooter/score", score);
                });
        intakeSimulation.removeFuel(1);
        simulatedArena.addGamePieceProjectile(fuelOnFly);
    }

    public Command shootSim() {
        return Commands.parallel(
                commandGroupFactory.prepareToShoot(true),
                Commands.run(
                        () -> Logger.recordOutput("Shooter/readyToShoot", commandGroupFactory.isReadyToShoot(true))),
                Commands.run(() -> {
                    if (commandGroupFactory.isReadyToShoot(true).getAsBoolean()
                            && intakeSimulation.getFuelCount() >= 1) {
                        shootSimulatedFuel();
                    }
                }));
    }

}
