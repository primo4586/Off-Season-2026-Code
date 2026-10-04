package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
//TODO: recalibrate based on new intake when new intake avaiable
public class IntakeArmConstants {
    //important
    public static final int MOTOR_ID = 21;


    //Locations
    // open and close postisions
        public static final Angle MIN_ANGLE_DEGREES = Degrees.of(0); 
        public static final Angle MAX_ANGLE_DEGREES = Degrees.of(693.36);
    //open and close with voltage
        public static final Voltage OPEN_VOLTAGE = Volts.of(1);
        public static final Voltage CLOSE_VOLTAGE = Volts.of(1);
        public static final Time OPEN_VOLTAGE_TIME = Seconds.of(1);
        public static final Time CLOSE_VOLTAGE_TIME = Seconds.of(1);
    //control logic 
    public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
    //Real PID constants 
    public static final double REAL_KP = 20;
    public static final double REAL_KI = 0.0;
    public static final double REAL_KD = 1;
    //Real FEEDFORWARD constants
    public static final double REAL_KS = 0;
    public static final double REAL_KV = 0;
    public static final double REAL_KG = 0;
    //SIM PID constants
    public static final double SIM_KP = 10;
    public static final double SIM_KI = 0.0;
    public static final double SIM_KD = 1;
     //SIM FEEDFORWARD constants
    public static final double SIM_KS = 0;
    public static final double SIM_KV = 0;
    public static final double SIM_KG = 0;
    //Physical Constants
    public static final double GEARING = 5;
    public static final boolean INVERTED = false;
    public static final MotorMode MOTOR_MODE= MotorMode.BRAKE;
    public static final double MAXVEL = 10;
    public static final double ACELERATION = 5;
    //LIMITS
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
    public static final Time RAMP_RATE = Seconds.of(0.2); 
    public static final Angle MID_POINT = Degrees.of(-360);
    public static final Distance ARM_LENGTH = Meters.of(0.56);
    //TOLERANCE
    public static final Angle ARM_TOLERANCE = Degrees.of(2);
    //Telematry
    public static final TelemetryVerbosity MOTOR_TELEMATRY_MODE = TelemetryVerbosity.HIGH;
    public static final TelemetryVerbosity ARM_TELEMETRY_MODE = TelemetryVerbosity.HIGH;
}
