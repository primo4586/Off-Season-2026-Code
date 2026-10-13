package frc.robot.primoLib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PrimoCalc {
  private static final Drive drive = Drive.getInstance(RobotBase.isReal());

  /** @return the robot's current pose as a Pose2d. */
  public static Pose2d getRobotPose() {
    return drive.getPose();
  }

  /**
   * Snaps an angle to the nearest value of the form 45 + 90k and normalizes the result to the range
   * [0, 360].
   *
   * @param angle input angle in degrees
   * @return nearest snapped angle in [0, 360]
   */
  public static double bumpAngle(double angle) {
    double snapped = 45 + 90 * Math.round((angle - 45) / 90.0);
    return (snapped % 360 + 360) % 360;
  }

  /**
   * @return hub coordinates as a pose3d
   */
  public static Pose3d getHubPos() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue
        ? new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero)
        : new Pose3d(11.912, Units.inchesToMeters(158.84),
        Units.inchesToMeters(72), Rotation3d.kZero);
  }

    /**
   * @return get distance in meters from robot to ds wall
   */
  public static DoubleSupplier getDistanceFromDSWall() {
    return ()-> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue
        ? getRobotPose().getTranslation().getX()
        : 16.5 - getRobotPose().getTranslation().getX();
  }

  //returns the direction the robot should be facing to face the ds wall
  public static Rotation2d getRotToDsWall(){
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue
        ? new Rotation2d(Math.PI)
        : new Rotation2d();
  }

  public static BooleanSupplier pastMiddle(){
    return () -> getDistanceFromDSWall().getAsDouble() > 8.5;
  }

  //pass values
  public static DoubleSupplier shooterPassVel(double closerThanMidRPS, double furtherThanMidRPS){
    return ()-> pastMiddle().getAsBoolean() ? furtherThanMidRPS : closerThanMidRPS;
  }

  public static DoubleSupplier hoodPassPos(double closerThanMidPos, double furtherThanMidPos){
  return ()-> pastMiddle().getAsBoolean() ? furtherThanMidPos : closerThanMidPos;
  }

  /**
   * @return the angle the robot needs to be to point at the hub, on radians.
   */
  public static double getRadsToHub() {
    return drive
            .getPose()
            .getTranslation()
            .minus(getHubPos().toPose2d().getTranslation())
            .getAngle()
            .getRadians()
        + Math.PI;
  }

  /**
   * @param targetPose the pose to point at
   * @return returns the angle the robot needs to be to point at a given pose, in radians.
   */
  public static double getRadsToPose(Pose2d targetPose) {
    return drive
            .getPose()
            .getTranslation()
            .minus(targetPose.getTranslation())
            .getAngle()
            .getRadians()
        + Math.PI;
  }

/**
 * Checks if the robot is facing the hub within a certain threshold.
 */
  public static BooleanSupplier isFacingHub() {
    return () ->
        Math.abs(getRadsToHub() - drive.getRotation().getRadians())
            < Constants.HUB_ANGLE_RADS_THRESHOLD;
  }

  public static double getDistance(Pose2d a, Pose3d b) {
    return a.getTranslation().getDistance(b.toPose2d().getTranslation());
  }





  
}
