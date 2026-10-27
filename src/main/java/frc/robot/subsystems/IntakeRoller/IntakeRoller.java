package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.subsystems.IntakeRoller.IntakeRollerConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeRoller extends SubsystemBase {
  @AutoLog
  public static class IntakeRollerInputs {
    public double velocity = 0; // RPM
    public double setpoint = 0;
    public Voltage volts = Volts.of(0);
    public Current supplyCurrent = Amps.of(0);
    public Current statorCurrent = Amps.of(0);
    public double temperature = 0.0; // Celsius
    public boolean isTuning = false;
  }

  private final IntakeRollerInputsAutoLogged m_inputs = new IntakeRollerInputsAutoLogged();

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(CONTROL_MODE)
      .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
      .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
      .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
      .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
      .withTelemetry(MOTOR_TELEMETRY_NAME, MOTOR_TELEMATRY_MODE)
      .withGearing(GEARING)
      .withMotorInverted(INVERTED)
      .withIdleMode(MOTOR_MODE)
      .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
      .withStatorCurrentLimit(SUPPLY_CURRENT_LIMIT);

  private TalonFX talonFX = new TalonFX(MOTOR_ID);
  private SmartMotorController motor = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), smcConfig);
  private final FlyWheelConfig intakeRollerConfig = new FlyWheelConfig()
      .withDiameter(Inch.of(2))
      // Telemetry name and verbosity for the arm.
      .withTelemetry(SUBSYSTEM_TELEMETRY_NAME, FLYWHEEL_TELEMATRY_MODE);
  private FlyWheel intakeRoller = new FlyWheel(intakeRollerConfig, motor);
    NetworkTable tuningTable = NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getSubTable(SUBSYSTEM_TELEMETRY_NAME).getSubTable(MOTOR_TELEMETRY_NAME);

    public IntakeRoller() {
        tuningTable.getEntry("voltage/enabled").setBoolean(false);
        tuningTable.getEntry("voltage/setpoint").setDouble(0.0);
    }
  public void updateInputs() {
    m_inputs.velocity = intakeRoller.getSpeed().in(RPM);
    m_inputs.setpoint = intakeRoller.getMechanismSetpointVelocity().orElse(RPM.of(0)).in(RPM);
    m_inputs.volts = intakeRoller.getMotor().getVoltage();
    m_inputs.statorCurrent = intakeRoller.getMotor().getStatorCurrent();
    m_inputs.statorCurrent = intakeRoller.getMotor().getSupplyCurrent().orElse(Amps.of(0));
    m_inputs.temperature = intakeRoller.getMotor().getTemperature().in(Celsius);
            m_inputs.isTuning = SmartDashboard
                .getBoolean("Mechanisms/Commands/" + SUBSYSTEM_TELEMETRY_NAME + "/Live Tuning/running", false);
        if (tuningTable.getEntry("voltage/enabled").getBoolean(false) && m_inputs.isTuning) {
            intakeRoller.setVoltageSetpoint(Volts.of(tuningTable.getEntry("voltage/setpoint").getDouble(0)));
        }
  }

  /**
   * a command that runs intakes with a preSelected speed
   * 
   * @return
   */
  public Command intake() {
    return intakeRoller.run(RPM.of(INTAKE_SPEED));
  }

  /**
   * a command that runs outakes with a preSelected speed
   * 
   * @return
   */
  public Command outake() {
    return intakeRoller.run(RPM.of(-INTAKE_SPEED));
  }

  /**
   * a command that runs intakes with a preSelected voltage
   * 
   * @return
   */
  public Command intakeWithVoltage() {
    return intakeRoller.setVoltage(Volts.of(INTAKE_VOLTAGE));
  }

  /**
   * a command that runs outakes with a preSelected voltage
   * 
   * @return
   */
  public Command outakeWithVoltage() {
    return intakeRoller.setVoltage(Volts.of(-INTAKE_VOLTAGE));
  }

  /**
   * a command that runs intakes with a Selected speed
   * 
   * @param intakeVelocity - desired AngularVelocity
   * @return
   */
  public Command intake(AngularVelocity intakeVelocity) {
    return intakeRoller.run(intakeVelocity);
  }

  /**
   * a command that runs outakes with a Selected voltage
   * 
   * @param intakeVoltage - desired voltage
   * @return
   */
  public Command intakeWithVoltage(Voltage intakeVoltage) {
    return intakeRoller.setVoltage(intakeVoltage);
  }

  /**
   * gives current spped of motor
   * 
   * @return AngularVelocity of motor
   */
  public AngularVelocity getVelocity() {
    return intakeRoller.getSpeed();
  }

  /**
   * a command that runs the motor at desired precent of its maximum power until
   * interrupted
   * 
   * @param dutyCylce - a number in [-1,1] that determines the precnet of the
   *                  maximum power that the motor will run at
   * @return
   */
  public Command set(double dutyCycle) {
    return intakeRoller.set(dutyCycle);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(SUBSYSTEM_TELEMETRY_NAME, m_inputs);
    intakeRoller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakeRoller.simIterate();
  }

}
