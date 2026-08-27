package frc.robot.subsystems.IntakeRoller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
public class IntakeRollerConstants {
    public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
    //Real PID constants
    public static final float REAL_KP = 50;
    public static final float REAL_KI = 0;
    public static final float REAL_KD = 0;
     //Real FEEDFORWARD constants
    public static final float REAL_KS = 0;
    public static final float REAL_KV = 0;
    public static final float REAL_KA = 0;
    //SIM PID constants
    public static final float SIM_KP = 50;
    public static final float SIM_KI = 0;
    public static final float SIM_KD = 0;
     //SIM FEEDFORWARD constants
    public static final float SIM_KS = 0;
    public static final float SIM_KV = 0;
    public static final float SIM_KA = 0;
    //Physical Constants
    public static final double GEARING = 3;
    public static final boolean INVERTED = false;
    public static final MotorMode MOTOR_MODE= MotorMode.COAST;
    public static final int MOTOR_ID = 2;
    public static final int INTAKE_SPEED = 60;
    //LIMITS
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
    //Telematry
    public static final TelemetryVerbosity MOTOR_TELEMATRY_MODE = TelemetryVerbosity.HIGH;
    public static final TelemetryVerbosity FLYWHEEL_TELEMATRY_MODE = TelemetryVerbosity.HIGH;
    
    
}
