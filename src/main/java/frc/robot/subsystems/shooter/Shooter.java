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
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

import javax.sound.sampled.Line;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {

    @AutoLog
    public static class ShooterInputs {
        public double angularVelocity = 0;
        public double acceleration = 0;
        public double setpoint = 0;
        public Voltage volts = Volts.of(0);
        public Current statorcurrent = Amps.of(0);
        public Current supplycurrent = Amps.of(0);
    }

    private final ShooterInputsAutoLogged m_inputs = new ShooterInputsAutoLogged();
    @SuppressWarnings("removal")
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withMechanismCircumference(WHEEL_CIRCUMFERENCE)
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
    @SuppressWarnings("removal")
    private TalonFX talonFX = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    private SmartMotorController motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(2), smcConfig);
    private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
            .withDiameter(Inches.of(2))
            .withTelemetry("Shooter", MECHANISM_VERBOSITY);
    private FlyWheel shooter = new FlyWheel(shooterConfig, motor);

    private void updateInputs() {
        m_inputs.angularVelocity = shooter.getSpeed().in(RPM);
        m_inputs.acceleration = shooter.getMotor().getMeasurementAcceleration().in(MetersPerSecondPerSecond);
        m_inputs.setpoint = shooter.getMechanismSetpointVelocity().orElse(RPM.of(0)).in(RPM);
        m_inputs.volts = shooter.getMotor().getVoltage();
        m_inputs.statorcurrent = shooter.getMotor().getStatorCurrent();
        var supplyCurrent = motor.getSupplyCurrent();
        if (supplyCurrent.isPresent()) {
            m_inputs.supplycurrent = supplyCurrent.get();
        } else {
            System.err.println("unable to get supplycurrent autologger not updated");
        }

    }

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return RPM.of(m_inputs.angularVelocity);
    }

    /**
     * Gets the current acceleration of the main shooter motor.
     *
     * @return Shooter Acceleration.
     */
    public LinearAcceleration getAcceleration() {
        return MetersPerSecondPerSecond.of(m_inputs.acceleration);
    }

    /**
     * Gets the current voltage of the main shooter motor.
     *
     * @return Shooter Voltage.
     */
    public Voltage getVoltage() {
        return m_inputs.volts;
    }

    /**
     * Gets the current stator current of the main shooter motor.
     *
     * @return Shooter stator current.
     */
    public Current getStatorCurrent() {
        return m_inputs.statorcurrent;
    }

    /**
     * Gets the current supply current of the main shooter motor.
     *
     * @return Shooter supply current.
     */
    public Current getSupplyCurrent() {
        return m_inputs.statorcurrent;
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
        updateInputs();
        Logger.processInputs("Shooter", m_inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }

}