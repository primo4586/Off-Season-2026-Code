package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static frc.robot.subsystems.IntakeRoller.IntakeRollerConstants.*;

public class IntakeRoller extends SubsystemBase {
     private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
     .withControlMode(CONTROL_MODE)
     .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
  .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
  .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
  .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
  .withTelemetry("ShooterMotor", TELEMATRY_MODE)
  .withGearing(GEARING)
  .withMotorInverted(INVERTED)
  .withIdleMode(MOTOR_MODE)
  .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
  .withStatorCurrentLimit(SUPPLY_CURRENT_LIMIT);
}
