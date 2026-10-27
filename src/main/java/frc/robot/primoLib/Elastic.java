package frc.robot.primoLib;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elastic {
  static Field2d m_field = new Field2d();
  private static double simMatchTime = -1;
  public static boolean blueWon = false;

  public static void setSimMatchTime(double t) {
    simMatchTime = t;
  }

  public static double getMatchTime() {
    return (RobotBase.isSimulation() && simMatchTime >= 0)
        ? simMatchTime
        : Timer.getMatchTime();
  }
  // public static RobotState rState = RobotState.getInstance();

  public static void displayAll() {
    displayTimer();
    displayHubStatus();
    displayShiftTimer();
  }

  public static void displayTimer() {
    SmartDashboard.putNumber("timer", Math.round(getMatchTime())); // return match time rounded
  }

  public static void displayField() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Calculates and displays the remaining time in the current 2026 REBUILT
   * alliance shift,
   * or the time remaining in the match during End Game.
   */
  public static void displayShiftTimer() {
    double matchTime = getMatchTime();
    double timeUntilNextShift = 0.0;
    boolean isRedActive = false;
    boolean isBlueActive = false;
    int shiftNumber = 0;
    if (DriverStation.isAutonomous()) {
      // During Auto, show time until Teleop initialization
      timeUntilNextShift = matchTime;
      isBlueActive = true;
      isRedActive = true;
      shiftNumber = 0;

    } else if (DriverStation.isTeleop()) {
      if (matchTime > 130) {
        // Transition Shift (140s -> 130s)
        shiftNumber = 1;
        timeUntilNextShift = matchTime - 130;
        isBlueActive = true;
        isRedActive = true;
      } else if (matchTime > 105) {
        // Alliance Shift 1 (130s -> 105s)
        shiftNumber = 2;
        timeUntilNextShift = matchTime - 105;
        isBlueActive = !blueWon;
        isRedActive = blueWon;
      } else if (matchTime > 80) {
        // Alliance Shift 2 (105s -> 80s)
        shiftNumber = 3;
        timeUntilNextShift = matchTime - 80;
        isBlueActive = blueWon;
        isRedActive = !blueWon;
      } else if (matchTime > 55) {
        // Alliance Shift 3 (80s -> 55s)
        shiftNumber = 4;
        timeUntilNextShift = matchTime - 55;
        isBlueActive = !blueWon;
        isRedActive = blueWon;
      } else if (matchTime > 30) {
        // Alliance Shift 4 (55s -> 30s)
        shiftNumber = 5;
        timeUntilNextShift = matchTime - 30;
        isBlueActive = blueWon;
        isRedActive = !blueWon;
      } else {
        // End Game (30s -> 0s) - Countdown until the match ends
        shiftNumber = 6;
        timeUntilNextShift = matchTime;
        isBlueActive = true;
        isRedActive = true;
      }
    }
    SmartDashboard.putNumber("Time Until Next Shift", Math.round(timeUntilNextShift));
    SmartDashboard.putNumber("Active Shift", shiftNumber);
    SmartDashboard.putBoolean("blueActive", isBlueActive);
    SmartDashboard.putBoolean("redActive", isRedActive);

  }

  /**
   * Pushes the active/inactive state of our alliance hub to the dashboard.
   */
  public static void displayHubStatus() {
    SmartDashboard.putBoolean("Hub Active", isHubActive());
  }

  /**
   * Evaluates if our alliance's hub is currently active based on 2026 Game Data.
   * Both hubs are active during Auto, the Transition Shift, and End Game.
   * During Alliance Shifts 1-4, the hubs alternate based on FMS game data.
   */
  public static boolean isHubActive() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      return false; // Safely return false if FMS alliance assignment is missing
    }
    Alliance ourAlliance = allianceOpt.get();

    // Both hubs are fully functional during Autonomous
    if (DriverStation.isAutonomous()) {
      return true;
    }

    double matchTime = getMatchTime();

    // Transition Shift (140s -> 130s) and End Game (30s -> 0s) both hubs are active
    if (matchTime > 130 || matchTime <= 30) {
      return true;
    }

    // Retrieve FMS Game Data to see who goes inactive first
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
      return true; // Fallback to true if data hasn't arrived early in Teleop
    }

    char inactiveFirstChar = gameData.toUpperCase().charAt(0);
    boolean isOurAllianceInactiveFirst = (ourAlliance == Alliance.Red && inactiveFirstChar == 'R') ||
        (ourAlliance == Alliance.Blue && inactiveFirstChar == 'B');

    // Odd shifts: Shift 1 (130s down to 105s) and Shift 3 (80s down to 55s)
    boolean isOddShift = (matchTime > 105 && matchTime <= 130) || (matchTime > 55 && matchTime <= 80);

    if (isOddShift) {
      // In odd shifts, the alliance that won auto has an inactive hub
      return !isOurAllianceInactiveFirst;
    } else {
      // In even shifts (Shifts 2 & 4), the active hub flips to the opposite alliance
      return isOurAllianceInactiveFirst;
    }
  }
}
