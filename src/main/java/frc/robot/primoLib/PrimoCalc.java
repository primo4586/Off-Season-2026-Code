package frc.robot.primoLib;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.InterpolationMaps;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.interpolation.ExterpolationMap;
import frc.robot.util.interpolation.InterpolationMap;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class PrimoCalc {
  private final Drive drive;
  private final Alliance alliance;

  public PrimoCalc(Drive drive, Alliance allianceStationID) {
    this.drive = drive;
    this.alliance = allianceStationID;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  /** @return the robot's current pose as a Pose2d. */
  public Pose2d getRobotPose() {
    return drive.getPose();
  }

  /**
   * Snaps an angle to the nearest value of the form 45 + 90k and normalizes the
   * result to the range
   * [0, 360].
   *
   * @param angle input angle in degrees
   * @return nearest snapped angle in [0, 360]
   */
  public double bumpAngle(double angle) {
    double snapped = 45 + 90 * Math.round((angle - 45) / 90.0);
    return (snapped % 360 + 360) % 360;
  }

  /**
   * @return hub coordinates as a pose3d
   */
  public Pose3d getHubPos() {
    return getAlliance() == Alliance.Blue
        ? new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
            Units.inchesToMeters(72), Rotation3d.kZero)
        : new Pose3d(11.912, Units.inchesToMeters(158.84),
            Units.inchesToMeters(72), Rotation3d.kZero);
  }

  /**
   * @return get distance in meters from robot to ds wall
   */
  public DoubleSupplier getDistanceFromDSWall() {
    return () -> DriverStation.getAlliance().isPresent()
        && getAlliance() == Alliance.Blue
            ? getRobotPose().getTranslation().getX()
            : 16.5 - getRobotPose().getTranslation().getX();
  }

  // returns the direction the robot should be facing to face the ds wall
  public Rotation2d getRotToDsWall() {
    return DriverStation.getAlliance().isPresent()
        && getAlliance() == Alliance.Blue
            ? new Rotation2d(Math.PI)
            : new Rotation2d();
  }

  public BooleanSupplier pastMiddle() {
    return () -> getDistanceFromDSWall().getAsDouble() > 8.5;
  }

  // pass values
  public DoubleSupplier shooterPassVel(double closerThanMidRPS, double furtherThanMidRPS) {
    return () -> pastMiddle().getAsBoolean() ? furtherThanMidRPS : closerThanMidRPS;
  }

  public DoubleSupplier hoodPassPos(double closerThanMidPos, double furtherThanMidPos) {
    return () -> pastMiddle().getAsBoolean() ? furtherThanMidPos : closerThanMidPos;
  }

  /**
   * @return the angle the robot needs to be to point at the hub, on radians.
   */
  public double getRadsToHub() {
    return drive
        .getPose()
        .getTranslation()
        .minus(getHubPos().toPose2d().getTranslation())
        .getAngle()
        .getRadians();
  }

  /**
   * @param targetPose the pose to point at
   * @return returns the angle the robot needs to be to point at a given pose, in
   *         radians.
   */
  public double getRadsToPose(Pose2d targetPose) {
    return drive
        .getPose()
        .getTranslation()
        .minus(targetPose.getTranslation())
        .getAngle()
        .getRadians()
        + Math.PI;
  }
private static final Rotation2d SHOOTER_FACING_OFFSET = Rotation2d.fromDegrees(0);
private static final Rotation2d FACING_TOLERANCE = Rotation2d.fromDegrees(5);
  public Rotation2d getTargetHeading(Translation2d robotPos) {
    return getHubPos().toPose2d().getTranslation()
        .minus(robotPos)
        .getAngle()
        .plus(SHOOTER_FACING_OFFSET);
  }

  public boolean isFacingHub(Pose2d truePose) {
    Rotation2d error = truePose.getRotation()
        .minus(getTargetHeading(truePose.getTranslation()));
    Logger.recordOutput("test",error.getRadians());
    Logger.recordOutput("test1",FACING_TOLERANCE.getRadians());
    return Math.abs(error.getRadians()) < FACING_TOLERANCE.getRadians();
  }

  public double getDistance(Pose2d a, Pose3d b) {
    return a.getTranslation().getDistance(b.toPose2d().getTranslation());
  }

  public double interplateFromHub(ExterpolationMap interplationMap) {
    return interplationMap.exterpolate(getDistance(drive.getPose(), getHubPos()));
  }

  public Supplier<AngularVelocity> interplateShooter() {
    return () -> RotationsPerSecond.of(interplateFromHub(InterpolationMaps.SHOOTER_INTERPOLATION_MAP));
  }

  public Supplier<Angle> interplateHood() {
    return () -> Rotations.of(interplateFromHub(InterpolationMaps.HOOD_ANGLE_INTERPOLATION_MAP));
  }

  public Rotation2d getAngleToHub(Translation2d robotPos) {
    return getHubPos().toPose2d().getTranslation()
        .minus(robotPos)
        .getAngle();
  }
}
