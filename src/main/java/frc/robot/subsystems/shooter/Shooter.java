package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(CONTROL_MODE)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
            .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
            .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
            // Telemetry name and verbosity level
            .withTelemetry("ShooterMotor", MOTOR_VERBOSITY)
            .withGearing(GEARING)
            .withMotorInverted(INVERTED)
            .withIdleMode(NEUTRAL_MODE)
            .withStatorCurrentLimit(STATOR_LIMIT)
            .withSupplyCurrentLimit(SUPPLY_LIMIT)
            .withFollowers(Pair.of(new TalonFX(FOLLOWER_ID, Constants.CAN_BUS_NAME), FOLLOWER_INVERTED));
    private TalonFX talonFX = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    private SmartMotorController motor = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), smcConfig);
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
            .withDiameter(Inches.of(2))
            .withTelemetry("Shooter", MECHANISM_VERBOSITY);
    private FlyWheel shooter = new FlyWheel(shooterConfig, motor);

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Gets the current acceleration of the main shooter motor.
     *
     * @return Shooter Acceleration.
     */
    public LinearAcceleration getAcceleration() {
        return shooter.getMotor().getMeasurementAcceleration();
    }

    /**
     * Gets the current voltage of the main shooter motor.
     *
     * @return Shooter Voltage.
     */
    public Voltage getVoltage() {
        return shooter.getMotor().getVoltage();
    }

    /**
     * Gets the current stator current of the main shooter motor.
     *
     * @return Shooter stator current.
     */
    public Current getStatorCurrent() {
        return shooter.getMotor().getStatorCurrent();
    }

    /**
     * Gets the current supply current of the main shooter motor.
     *
     * @return Shooter supply current.
     */
    public Optional<Current> getSupplyCurrent() {
        return shooter.getMotor().getSupplyCurrent();
    }

    /**
     * runs shooter at velocitty
     * 
     * @param velocity velocity in AngularVelocity to run shooter at
     * @return Shooter velocity.
     */
    public Command run(AngularVelocity velocity) {
        return shooter.run(velocity);
    }

    /**
     * runs shooter at velocitty
     * 
     * @param velocity velocity in Supplier<AngularVelocity> to run shooter at
     * @return Shooter velocity.
     */
    public Command run(Supplier<AngularVelocity> velocity) {
        return shooter.run(velocity);
    }

    /**
     * sets intakeroller speed to dutyCycle
     * 
     * @param dutyCycle the speed to run the intakeroller at
     * @return Command
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    /**
     * Set the shooter motors to the given voltage.
     *
     * @param voltage the voltage to set the motor to (in volts)
     * @return a command which sets the voltage
     */
    public Command setVoltage(Voltage voltage) {
        return shooter.setVoltage(voltage);
    }

    /**
     * @return a command that sets velocity with SHOOT_SPEED constant
     */
    public Command shoot() {
        return shooter.run(SHOOT_SPEED);
    }

    /**
     * pass balls
     * 
     * @return a command that sets velocity with PASS_SPEED constant
     */
    public Command pass() {
        return shooter.run(PASS_SPEED);
    }

    /**
     * stop shooter
     * 
     * @return stops shooter by setting dutycycle to 0
     */
    public Command stop() {
        return shooter.set(0);
    }

    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }

}