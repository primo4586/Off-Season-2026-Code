package frc.robot.subsystems.intakeroller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.motorcontrollers.SmartMotorControllerConfig;
import static frc.robot.subsystems.intakeroller.IntakeRollerConstants.*;
public class IntakeRoller extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(CONTROL_MODE)
    // Feedback Constants (PID Constants)
    .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
    .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
    // Feedforward Constants
    .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
    .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
    // Telemetry name and verbosity level
    .withTelemetry("IntakeRollerMotor", VERBOSITY)
    .withGearing(GEARING)
    .withMotorInverted(INVERTED)
    .withIdleMode(NEUTRAL_MODE)
    .withStatorCurrentLimit(STATOR_LIMIT)
    .withSupplyCurrentLimit(SUPPLY_LIMIT);


}
