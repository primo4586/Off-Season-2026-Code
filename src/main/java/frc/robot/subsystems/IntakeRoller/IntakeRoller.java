package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

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
    * a command that runs intakes with a preSelected speed
    * @return
    */
   public Command intake() {return intake.run(RPM.of(INTAKE_SPEED));}
   /**
    * a command that runs outakes with a preSelected speed
    * @return
    */
   public Command outake(){return intake.run(RPM.of(-INTAKE_SPEED));}

     /**
    * a command that runs intakes with a preSelected voltage
    * @return
    */
   public Command intakeWithVoltage(){return intake.setVoltage(Volts.of(INTAKE_VOLTAGE));}
   
     /**
    * a command that runs outakes with a preSelected voltage
    * @return
    */
   public Command outakeWithVoltage(){return intake.setVoltage(Volts.of(-INTAKE_VOLTAGE));}
 /**
    * a command that runs intakes with a Selected speed
    * @param intakeVelocity - desired AngularVelocity
    * @return
    */
   public Command intake(AngularVelocity intakeVelocity) {return intake.run(intakeVelocity);}
   /**
    * a command that runs outakes with a Selected voltage
    * @param intakeVoltage - desired voltage
    * @return
    */
   public Command intakeWithVoltage(Voltage intakeVoltage){return intake.setVoltage(intakeVoltage);}
   
    
   /**
    * gives current spped of motor
    * @return AngularVelocity of motor
    */
    public AngularVelocity getVelocity(){return intake.getSpeed();}
    
     /**
    * a command that runs the motor at desired precent of its maximum power until interrupted
    * @param dutyCylce - a number in [-1,1] that determines the precnet of the maximum power that the motor will run at  
    * @return 
    */
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
