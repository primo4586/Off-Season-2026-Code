package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CanIdManager;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static frc.robot.subsystems.IntakeArm.IntakeArmConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeArm extends SubsystemBase {
    @AutoLog
    public static class IntakeArmInputs {
        public double setpoint = 0.0;
        public double position = 0.0;
        public double velocity = 0.0; // RPM
        public Voltage volts = Volts.of(0);
        public Current statorCurrent = Amps.of(0);
        public Current supplyCurrent = Amps.of(0);
        public Pose3d componentPose = new Pose3d();
    }

    private final IntakeArmInputsAutoLogged m_inputs;
    private SmartMotorControllerConfig smcConfig;
    private static TalonFX talonFX;
    private SmartMotorController motor;
    private ElevatorConfig armCfg;
    private Elevator arm;
    private final int id;
    public IntakeArm(int id, String name) {
        super(name);
        this.id = id;
        m_inputs = new IntakeArmInputsAutoLogged();
        smcConfig = new SmartMotorControllerConfig(this)
                .withMechanismCircumference(CIRCUMFERENCE)
                .withControlMode(CONTROL_MODE)
                // Feedback Constants (PID Constants)
                .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
                .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
                // Feedforward Constants
                .withFeedforward(new ElevatorFeedforward(REAL_KS, 0, REAL_KV))
                .withSimFeedforward(new ElevatorFeedforward(SIM_KS, 0, SIM_KV))
                // Telemetry name and verbosity level
                .withTelemetry("ArmMotor"+String.valueOf(id), MOTOR_TELEMATRY_MODE)
                // Gearing from the motor rotor to final shaft.
                // In this example GearBox.fromReductionStages(3,4) is the same as
                // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                // your motor.
                // You could also use .withGearing(12) which does the same thing.
                .withGearing(new MechanismGearing(GEARING))
                // Motor properties to prevent over currenting.
                .withMotorInverted(INVERTED)
                .withIdleMode(MOTOR_MODE)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withStartingPosition(MIN_DISTANCE_MM);

        talonFX = new TalonFX(CanIdManager.get(id,MOTOR_ID));
        motor = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), smcConfig);
        armCfg = new ElevatorConfig()
                .withHardLimits(MIN_DISTANCE_MM, MAX_DISTANCE_MM)
                .withCarriageWeight(CARRIAGE_WEIGHT)
                .withHorizontalElevator()
                .withAngle(Degrees.of(0))
                .withTelemetry("intakeArm"+String.valueOf(id), ARM_TELEMETRY_MODE);

        // Arm Mechanism
        arm = new Elevator(armCfg, motor);
    }

    public void updateInputs() {
        m_inputs.setpoint = smcConfig.convertFromMechanism(arm.getMechanismSetpoint().orElse(Degrees.of(0)))
                .in(Millimeter); // get setpoint in mm
        m_inputs.position = arm.getHeight().in(Millimeter);
        m_inputs.velocity = arm.getMotor().getMechanismVelocity().in(RPM);
        m_inputs.volts = arm.getMotor().getVoltage();
        m_inputs.statorCurrent = arm.getMotor().getStatorCurrent();
        m_inputs.statorCurrent = arm.getMotor().getSupplyCurrent().orElse(Amps.of(0));
        double totalExtensionMeters = (m_inputs.position / 1000.0);
        double deltaX = totalExtensionMeters * Math.cos(TRACK_ANGLE_DEGREES);
        double deltaZ = totalExtensionMeters * Math.sin(TRACK_ANGLE_DEGREES);
        m_inputs.componentPose = new Pose3d(
                new Translation3d(deltaX, 0.0, deltaZ), // Updates forward (X) and down (Z) simultaneously
                new Rotation3d() // No rotation
        );
    }

    /**
     * Sets the intake to positon.
     *
     * @param distance the distance to move.
     * @return the resulting command instance.
     */
    public Command SetIntakeToPositon(Distance distance) {
        return arm.runTo(distance, ARM_TOLERANCE);

    }

    /**
     * Opens the intake.
     *
     * @return the resulting command instance.
     */
    public Command OpenIntake() {

        return SetIntakeToPositon(MAX_DISTANCE_MM).andThen(() -> System.out.println("OPENING INTAKE"));
    }

    /**
     * Closes the intake.
     *
     * @return the resulting command instance.
     */
    public Command CloseIntake() {

        return SetIntakeToPositon(MIN_DISTANCE_MM);
    }

    /**
     * Opens the and close intake inorder to move the balls.
     *
     * @return the resulting command instance.
     */
    public Command OpenAndCloseIntake() {

        return Commands.repeatingSequence(arm.run(MIN_DISTANCE_MM).withTimeout(0.2),
                Commands.waitTime(Seconds.of(0.2)),
                arm.run(MID_POINT).withTimeout(Seconds.of(0.2)), Commands.waitTime(Seconds.of(0.2)));

    }

    /**
     * Sets the voltage.
     *
     * @param voltage the voltage.
     * @return the resulting command instance.
     */
    public Command setVoltage(Voltage voltage) {
        return arm.setVoltage(voltage);
    }

    /**
     * Opens the intake with voltage.
     *
     * @return the resulting command instance.
     */
    public Command openWithVoltage() {
        return setVoltage(OPEN_VOLTAGE).withTimeout(OPEN_VOLTAGE_TIME);
    }

    /**
     * Closes the intake with voltage.
     *
     * @return the resulting command instance.
     */
    public Command closeWithVoltage() {
        return setVoltage(CLOSE_VOLTAGE).withTimeout(CLOSE_VOLTAGE_TIME);
    }

    /**
     * Move the arm up and down.
     * 
     * @param dutycycle [-1, 1] speed to set the arm too.
     * @return the resulting command instance
     */
    public Command set(double dutycycle) {
        return arm.set(dutycycle);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("IntakeArm"+String.valueOf(id), m_inputs);
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }

}
