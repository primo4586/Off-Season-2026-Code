package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.InterpolationMaps;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.IntakeArm.IntakeArm;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class CommandGroupFactory {
    private final Drive drive;
    private final Feeder feeder;
    private final Hood hood;
    private final IntakeArm intakeArm;
    private final IntakeRoller intakeRoller;
    private final Shooter shooter;

    private final Command feederDefaultCommand;
    private final Command hoodDefaultCommand;
    private final Command intakeArmDefaultCommand;
    private final Command intakeRollerDefaultCommand;
    private final Command shooterDefaultCommand;

    private final DoubleSupplier driveXSupplier;
    private final DoubleSupplier driveYSupplier;

    /**
     * creates commandgroup factory powered by the given subsystems
     * NOTE: create after adding defualtcommands!
     * 
     * @param feeder
     * @param hood
     * @param intakeArm
     * @param intakeRoller
     * @param shooter
     */
    public CommandGroupFactory(Drive drive, Feeder feeder, Hood hood, IntakeArm intakeArm, IntakeRoller intakeRoller,
            Shooter shooter, DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
        this.drive = drive;
        this.feeder = feeder;
        this.hood = hood;
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        this.shooter = shooter;
        feederDefaultCommand = feeder.getDefaultCommand();
        hoodDefaultCommand = hood.getDefaultCommand();
        intakeArmDefaultCommand = intakeArm.getDefaultCommand();
        intakeRollerDefaultCommand = intakeRoller.getDefaultCommand();
        shooterDefaultCommand = shooter.getDefaultCommand();

        this.driveXSupplier = driveXSupplier;
        this.driveYSupplier = driveYSupplier;
    }

    /**
     * @return command that causes shooter and hood to interplate based on distance from hub
     */
    public Command interplateShooterAndHood() {
        return Commands.parallel(
                shooter.run(PrimoCalc.interplateShooter()),
                hood.run(PrimoCalc.interplateHood()));

    }

    /**
     * faces robot towards hub(if facehub == true), interplates shooter and hood,
     * activates feeder, and start opening and closing intake intake in order to
     * move fuels into the feeder
     * 
     * @param facehub if true shooting will only begin on facinghub and the robot
     *                will start attemping to face the hub.
     * @return commands that prepeares for shooting
     */
    public Command prepareToShoot(boolean facehub) {
        return Commands.parallel(
                Commands.sequence(
                        Commands.race(
                                Commands.waitUntil(() -> (PrimoCalc.isFacingHub().getAsBoolean() || !facehub) // checks
                                                                                                              // if
                                                                                                              // facing
                                                                                                              // hub
                                        && shooter.isReady().getAsBoolean()),
                                Commands.waitTime(Constants.MAXIMUM_SHOOT_WAIT_TIME)),
                        feeder.feed()),
                interplateShooterAndHood(),
                intakeArm.OpenAndCloseIntake(), // push fuels into feeder
                DriveCommands.joystickDriveAtAngle(
                        drive, () -> driveXSupplier.getAsDouble(), () -> driveYSupplier.getAsDouble(),
                        () -> new Rotation2d(PrimoCalc.getRadsToHub())).onlyIf(() -> facehub)); // look towards hub only
                                                                                                // if facehub == true
    }

    /**
     * simple shoot command with interplation
     * 
     * @return command that shoots
     */
    public Command shoot() {
        return Commands.parallel(
                prepareToShoot(true),
                shooter.shoot()

        );
    }

    /**
     * command to shoot when near hub
     * 
     * @param shootingVelocity the velocity to shoot at
     * @return command that shoots at shootingvelocity
     */
    public Command closeShoot(AngularVelocity shootingVelocity) {
        return Commands.parallel(
                prepareToShoot(false),
                shooter.run(shootingVelocity));
    }

    /**
     * shoot instantly without waiting to get to speed
     * 
     * @return command that immediatly shoots
     */
    public Command shootInstantly() {
        return Commands.parallel(
                feeder.feed(),
                intakeArm.OpenAndCloseIntake(), // push balls into feeder
                interplateShooterAndHood(),
                shooter.shoot());
    }

    /**
     * shoots at PASS_SPEED, used to pass balls to hub area from neutral zone
     * 
     * @return command that passes
     */
    public Command pass() {
        return Commands.parallel(
                prepareToShoot(false), // prepare to shoot without facing hub
                shooter.pass());
    }

    /**
     * stops all active commands on subsystems(except drive) and replaces them with
     * set(0)
     * 
     * @return command that stops all
     */
    public Command stopAll() {
        return Commands.parallel(feeder.set(0),
                hood.set(0),
                intakeArm.set(0),
                intakeRoller.set(0),
                shooter.run(RPM.of(0)));

    }

    /**
     * function that removes all defualt commands from all subsystems
     */
    public void removeDefualtCommands() {
        feeder.removeDefaultCommand();
        hood.removeDefaultCommand();
        intakeArm.removeDefaultCommand();
        intakeRoller.removeDefaultCommand();
        shooter.removeDefaultCommand();
    }

    /**
     * function that adds back all default command for all subsystems, based on the
     * defualt command at the time of factory creation.
     */
    public void addDefualtCommands() {
        feeder.setDefaultCommand(feederDefaultCommand);
        hood.setDefaultCommand(hoodDefaultCommand);
        intakeArm.setDefaultCommand(intakeArmDefaultCommand);
        intakeRoller.setDefaultCommand(intakeRollerDefaultCommand);
        shooter.setDefaultCommand(shooterDefaultCommand);
    }

    /**
     * @return command that removes all defualt commands from all subsystems
     */
    public Command removeDefualtCommandsCommand() {
        return Commands.runOnce(this::removeDefualtCommands);
    }

    /**
     * @return command that adds back all default command for all subsystems, based
     *         on the defualt command at the time of factory creation.
     */
    public Command addDefualtCommandsCommand() {
        return Commands.runOnce(this::addDefualtCommands);
    }

}
