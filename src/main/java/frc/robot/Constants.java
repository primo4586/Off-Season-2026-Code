// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.interpolation.ExterpolationMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String CAN_BUS_NAME = "canBus";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class InterpolationMaps {
    public static final double InterpolationSpeedMult = 0.97;
    public static final ExterpolationMap SHOOTER_INTERPOLATION_MAP = new ExterpolationMap()
        .put(0, 0)
        .put(1.485, 57.75 * InterpolationSpeedMult)
        .put(1.85, 60.9 * InterpolationSpeedMult)
        .put(2.095, 68.5 * InterpolationSpeedMult)
        .put(2.4, 72 * InterpolationSpeedMult)
        .put(2.6, 74 * InterpolationSpeedMult)
        .put(3.01, 75.5 * InterpolationSpeedMult)
        .put(3.48, 76.75 * InterpolationSpeedMult)
        .put(3.88, 79.5 * InterpolationSpeedMult);
    public static ExterpolationMap HOOD_ANGLE_INTERPOLATION_MAP = new ExterpolationMap()
        .put(0, 0)
        .put(1.485, 0.01)
        .put(1.85, 0.01)
        .put(2.095, 0.011)
        .put(2.4, 0.012)
        .put(2.6, 0.01261)
        .put(3.01, 0.0135)
        .put(3.48, 0.14)
        .put(3.88, 0.15);
  }

  public static final double FIELD_LENGTH = 16.541;

  // Blue (faces +X, 0°)
  public static final Pose2d BLUE_LEFT = new Pose2d(3.0, 6.0, Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_CENTER = new Pose2d(3.0, 4.035, Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_RIGHT = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0));

  // Red (faces -X, 180°). Left/right swap in Y because the red driver looks the
  // other way
  public static final Pose2d RED_LEFT = new Pose2d(FIELD_LENGTH - 3.0, 2.0, Rotation2d.fromDegrees(180));
  public static final Pose2d RED_CENTER = new Pose2d(FIELD_LENGTH - 3.0, 4.035, Rotation2d.fromDegrees(180));
  public static final Pose2d RED_RIGHT = new Pose2d(FIELD_LENGTH - 3.0, 6.0, Rotation2d.fromDegrees(180));
  public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        RED_LEFT, RED_CENTER, RED_RIGHT, BLUE_LEFT, BLUE_CENTER,BLUE_RIGHT
};
public static final Alliance[] idToAliance = new Alliance[] {
  Alliance.Red, Alliance.Red, Alliance.Red, Alliance.Blue, Alliance.Blue,Alliance.Blue
};
  public static final Time SHOOT_TIMEOUT = Seconds.of(3);
  public static final Time MAXIMUM_SHOOT_WAIT_TIME = Seconds.of(0.8);
  public static final double HUB_ANGLE_RADS_THRESHOLD = 0.1;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
