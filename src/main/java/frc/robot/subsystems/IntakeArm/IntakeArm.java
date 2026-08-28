package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static frc.robot.subsystems.IntakeArm.IntakeArmConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeArm extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(CONTROL_MODE)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
            .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
            // Feedforward Constants
            .withFeedforward(new ArmFeedforward(REAL_KS, REAL_KG, REAL_KV))
            .withSimFeedforward(new ArmFeedforward(SIM_KS, SIM_KG, SIM_KV))
            // Telemetry name and verbosity level
            .withTelemetry("ArmMotor", MOTOR_TELEMATRY_MODE)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as
            // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
            // your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(GEARING)
            // Motor properties to prevent over currenting.
            .withMotorInverted(INVERTED)
            .withIdleMode(MOTOR_MODE)
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withStartingPosition(MIN_ANGLE_DEGREES);

    private static TalonFX talonFX = new TalonFX(MOTOR_ID);
    private SmartMotorController motor = new TalonFXWrapper(talonFX, DCMotor.getFalcon500(1), smcConfig);
    private ArmConfig armCfg = new ArmConfig()
            // Hard limit is applied to the simulation.
            .withHardLimits(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
            // Length and mass of your arm for sim.
            .withLength(ARM_LENGTH)
            // Telemetry name and verbosity for the arm.
            .withTelemetry("Arm", ARM_TELEMETRY_MODE);

    // Arm Mechanism
    private Arm arm = new Arm(armCfg, motor);
    

    public Command SetIntakeToPositon(Angle angle) {
        return arm.runTo(angle,ARM_TOLERANCE);

    }

    public Command OpenIntake() {
        
        return SetIntakeToPositon(MAX_ANGLE_DEGREES);
    }

    public Command CloseIntake() {
       
        return SetIntakeToPositon(MIN_ANGLE_DEGREES);
    }

    public Command OpenAndCloseIntake() {
       
        return Commands.repeatingSequence(arm.run(MIN_ANGLE_DEGREES).withTimeout(0.2), Commands.waitTime(Seconds.of(0.2)),
                        arm.run(MID_POINT).withTimeout(Seconds.of(0.2)), Commands.waitTime(Seconds.of(0.2)));
                
    }
     /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}

      @Override
    public void periodic(){
        arm.updateTelemetry();
    }
    @Override
    public void simulationPeriodic(){
        arm.simIterate();
    }

}
