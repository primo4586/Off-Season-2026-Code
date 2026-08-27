package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.RPM;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;

import static frc.robot.subsystems.IntakeRoller.IntakeRollerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeRoller extends SubsystemBase {
     private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
     .withControlMode(CONTROL_MODE)
     .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
  .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
  .withFeedforward(new SimpleMotorFeedforward(REAL_KS, REAL_KV, REAL_KA))
  .withSimFeedforward(new SimpleMotorFeedforward(SIM_KS, SIM_KV, SIM_KA))
  .withTelemetry("IntakeMotor", MOTOR_TELEMATRY_MODE)
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
  .withTelemetry("IntakeRollerMech", FLYWHEEL_TELEMATRY_MODE);
   private FlyWheel intake = new FlyWheel(intakeRollerConfig, motor);
   /**
    * a command that runs the motor at desired angular velocity
    * @param speed
    * @return
    */
   public Command intake() {return intake.run(RPM.of(INTAKE_SPEED));}
   public Command outake(){return intake.run(RPM.of(-INTAKE_SPEED));}
   /**
    * gives spped of motor
    * @return AngularVelocity of motor
    */
    public AngularVelocity getVelocity(){return intake.getSpeed();}
    
    public Command set(double dutyCycle){return intake.set(dutyCycle);}


    @Override
    public void periodic(){
        intake.updateTelemetry();
    }
    @Override
    public void simulationPeriodic(){
        intake.simIterate();
    }


 
}
