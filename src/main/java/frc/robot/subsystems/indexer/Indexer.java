package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class Indexer extends SubsystemBase {
    @AutoLog
    public static class IndexerInputs {

        public double velocity = 0.0; // RPM
        public Voltage volts = Volts.of(0);
        public Current statorCurrent = Amps.of(0);
        public Current supplyCurrent = Amps.of(0);
        public boolean isTuning = false;
    }

    private final IndexerInputsAutoLogged m_inputs = new IndexerInputsAutoLogged();
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withMechanismCircumference(CIRCUMFRANCE)
            .withControlMode(CONTROL_MODE)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
            .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
            .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
            // Telemetry name and verbosity level
            .withTelemetry(MOTOR_TELEMETRY_NAME, MOTOR_VERBOSITY)
            .withGearing(GEARING)
            .withMotorInverted(INVERTED)
            .withIdleMode(NEUTRAL_MODE)
            .withStatorCurrentLimit(STATOR_LIMIT)
            .withSupplyCurrentLimit(SUPPLY_LIMIT);

    private TalonFX talonFX = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    private SmartMotorController motor = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), smcConfig);
    private final FlyWheelConfig indexerConfig = new FlyWheelConfig()
            .withDiameter(Millimeters.of(40))
            .withTelemetry(SUBSYSTEM_TELEMETRY_NAME, MECHANISM_VERBOSITY);
    private FlyWheel indexer = new FlyWheel(indexerConfig, motor);
    NetworkTable tuningTable = NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getSubTable(SUBSYSTEM_TELEMETRY_NAME).getSubTable(MOTOR_TELEMETRY_NAME);

    public Indexer() {
        
    }

    private void updateInputs() {
        m_inputs.velocity = indexer.getMotor().getMechanismVelocity().in(RPM);
        m_inputs.volts = indexer.getMotor().getVoltage();
        m_inputs.statorCurrent = indexer.getMotor().getStatorCurrent();
        m_inputs.supplyCurrent = indexer.getMotor().getSupplyCurrent().orElse(Amps.of(0));
        m_inputs.isTuning = SmartDashboard
                .getBoolean("Mechanisms/Commands/" + SUBSYSTEM_TELEMETRY_NAME + "/Live Tuning/running", false);
        if (tuningTable.getEntry("voltage/enabled").getBoolean(false) && m_inputs.isTuning) {
            indexer.setVoltageSetpoint(Volts.of(tuningTable.getEntry("voltage/setpoint").getDouble(0)));
        }
    }

    /**
     * Gets the current velocity of the indexer.
     *
     * @return Indexer velocity.
     */
    public AngularVelocity getVelocity() {
        return RPM.of(m_inputs.velocity);
    }

    /**
     * Activates INDEXER with constant FEED_SPEED
     * in order to force balls into the feeder
     * 
     * @return Command
     */
    public Command feed() {
        return indexer.run(FEED_SPEED);
    }

    /**
     * Activates INDEXER with constant UNFEED_SPEED
     * in order to remove jammed balls from feeder
     * 
     * @return Command
     */
    public Command unfeed() {
        return indexer.run(UNFEED_SPEED);
    }

    /**
     * Activates INDEXER with constant FEED_VOLTAGE
     * in order to force balls into the feeder
     * 
     * @return Command
     */
    public Command feedWithVoltage() {
        return indexer.setVoltage(FEED_VOLTAGE);
    }

    /**
     * Activates INDEXER with constant UNFEED_VOLTAGE
     * in order to remove jammed balls from feeder
     * 
     * @return Command
     */
    public Command unfeedWithVoltage() {
        return indexer.setVoltage(UNFEED_VOLTAGE);
    }

    /**
     * sets indexer speed to dutyCycle
     * 
     * @param dutyCycle the speed to run the indexer at
     * @return Command
     */
    public Command set(double dutyCycle) {
        return indexer.set(dutyCycle);
    }

    /**
     * sets indexer voltage to voltage
     * 
     * @param voltage the voltage to run the indexer at
     * @return Command
     */
    public Command setVoltage(Voltage voltage) {
        return indexer.setVoltage(voltage);
    }

    /**
     * sets indexer velocity to velocity
     * 
     * @param velocity the velocity to run the indexer at
     * @return Command
     */
    public Command run(AngularVelocity velocity) {
        return indexer.run(velocity);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs(SUBSYSTEM_TELEMETRY_NAME, m_inputs);
        indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        indexer.simIterate();
    }
}
