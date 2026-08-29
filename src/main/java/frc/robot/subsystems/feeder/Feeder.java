package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class Feeder extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(CONTROL_MODE)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
            .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
            .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
            // Telemetry name and verbosity level
            .withTelemetry("FeederMotor", MOTOR_VERBOSITY)
            .withGearing(GEARING)
            .withMotorInverted(INVERTED)
            .withIdleMode(NEUTRAL_MODE)
            .withStatorCurrentLimit(STATOR_LIMIT)
            .withSupplyCurrentLimit(SUPPLY_LIMIT);

    private TalonFX talonFX = new TalonFX(MOTOR_ID);
    private SmartMotorController motor = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), smcConfig);
    private final FlyWheelConfig feederConfig = new FlyWheelConfig()
            .withDiameter(Inches.of(2))
            .withTelemetry("Feeder", MECHANISM_VERBOSITY);
    private FlyWheel feeder = new FlyWheel(feederConfig, motor);

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return feeder.getSpeed();
    }
    
    /**
     * Activates FEEDER with constant FEED_SPEED
     * in order to force balls into the shooter
     * 
     * @return Command
     */
    public Command feed() {
        return feeder.run(FEED_SPEED);
    }

    /**
     * Activates FEEDER with constant UNFEED_SPEED
     * in order to remove jammed balls from shooter
     * 
     * @return Command
     */
    public Command unfeed() {
        return feeder.run(UNFEED_SPEED);
    }
    
    /**
     * Activates FEEDER with constant FEED_VOLTAGE
     * in order to force balls into the shooter
     * 
     * @return Command
     */
    public Command feedWithVoltage() {
        return feeder.setVoltage(FEED_VOLTAGE);
    }
    
    /**
     * Activates FEEDER with constant UNFEED_VOLTAGE
     * in order to remove jammed balls from shooter
     * 
     * @return Command
     */
    public Command unfeedWithVoltage() {
        return feeder.setVoltage(UNFEED_VOLTAGE);
    }
    /**
     * sets intakeroller speed to dutyCycle
     * @param dutyCycle the speed to run the intakeroller at
     * @return Command
     */
  public Command set(double dutyCycle)
  {
    return feeder.set(dutyCycle);
  }

    @Override
    public void periodic() {
        feeder.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        feeder.simIterate();
    }
}
