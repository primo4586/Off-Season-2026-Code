package frc.robot.primoLib;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elastic {
  static Field2d m_field = new Field2d();
  // public static RobotState rState = RobotState.getInstance();

  public static void displayAll() {
    displayRobotPose();
    displayTimer();
    displayHubStatus();
    displayShiftTimer();
  }

  public static void displayTimer() {
    SmartDashboard.putNumber("timer", Timer.getMatchTime());
  }

  public static void displayField() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
  }

  public static void displayRobotPose() {
    m_field.setRobotPose(PrimoCalc.getRobotPose());
  }

  /**
   * Calculates and displays the remaining time in the current 2026 REBUILT alliance shift,
   * or the time remaining in the match during End Game.
   */
  public static void displayShiftTimer() {
    double matchTime = Timer.getMatchTime();
    double timeUntilNextShift = 0.0;

    if (DriverStation.isAutonomous()) {
      // During Auto, show time until Teleop initialization
      timeUntilNextShift = matchTime; 
    } 
    else if (DriverStation.isTeleop()) {
      if (matchTime > 130) {
        // Transition Shift (140s -> 130s)
        timeUntilNextShift = matchTime - 130;
      } else if (matchTime > 105) {
        // Alliance Shift 1 (130s -> 105s)
        timeUntilNextShift = matchTime - 105;
      } else if (matchTime > 80) {
        // Alliance Shift 2 (105s -> 80s)
        timeUntilNextShift = matchTime - 80;
      } else if (matchTime > 55) {
        // Alliance Shift 3 (80s -> 55s)
        timeUntilNextShift = matchTime - 55;
      } else if (matchTime > 30) {
        // Alliance Shift 4 (55s -> 30s)
        timeUntilNextShift = matchTime - 30;
      } else {
        // End Game (30s -> 0s) - Countdown until the match ends
        timeUntilNextShift = matchTime; 
      }
      SmartDashboard.putNumber("Time Until Next Shift", timeUntilNextShift);
    } 
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

    double matchTime = Timer.getMatchTime();

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
    boolean isOurAllianceInactiveFirst = 
        (ourAlliance == Alliance.Red && inactiveFirstChar == 'R') ||
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
