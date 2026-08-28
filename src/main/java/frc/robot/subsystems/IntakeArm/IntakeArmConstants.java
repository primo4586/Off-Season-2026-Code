package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import pabeles.concurrency.IntOperatorTask.Max;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class IntakeArmConstants {
    public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
    //Real PID constants
    public static final float REAL_KP = 1;
    public static final float REAL_KI = 0;
    public static final float REAL_KD = 0;
     //Real FEEDFORWARD constants
    public static final float REAL_KS = 0;
    public static final float REAL_KV = 0;
    public static final float REAL_KG = 0;
    //SIM PID constants
    public static final float SIM_KP = 9;
    public static final float SIM_KI = 1;
    public static final float SIM_KD = 1;
     //SIM FEEDFORWARD constants
    public static final float SIM_KS = 0;
    public static final float SIM_KV = 0;
    public static final float SIM_KG = 0;
    //Physical Constants
    public static final double GEARING = 3;
    public static final boolean INVERTED = false;
    public static final MotorMode MOTOR_MODE= MotorMode.BRAKE;
    public static final int MOTOR_ID = 7;
    public static final double MAXVEL = 10;
    public static final double ACELERATION = 5;
    //LIMITS
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
    public static final Time RAMP_RATE = Seconds.of(0.2); 
    public static final Angle MIN_ANGLE_DEGREES = Degrees.of(90); 
    public static final Angle MAX_ANGLE_DEGREES = Degrees.of(180);
    public static final Angle MID_POINT = Degrees.of((MIN_ANGLE_DEGREES.in(Degrees)+MAX_ANGLE_DEGREES.in(Degrees))/2);
    public static final Distance ARM_LENGTH = Meters.of(0.56);
    //TOLERANCE
    public static final Angle ARM_TOLERANCE = Degrees.of(2);
    //Telematry
    public static final TelemetryVerbosity MOTOR_TELEMATRY_MODE = TelemetryVerbosity.HIGH;
    public static final TelemetryVerbosity ARM_TELEMETRY_MODE = TelemetryVerbosity.HIGH;
}
