package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
//TODO: recalibrate based on new intake when new intake avaiable
public class IntakeArmConstants {
    //important
    public static final int MOTOR_ID = 21;
    public static final Mass CARRIAGE_WEIGHT = Grams.of(8802.11);
    public static final Distance CIRCUMFERENCE = Inches.of(5.875).times(Math.PI);
    //Locations
    // open and close postisions
        public static final Distance MIN_DISTANCE_MM = Millimeters.of(0); 
        public static final Distance MAX_DISTANCE_MM = Millimeters.of(311.37);
        public static final double TRACK_ANGLE_DEGREES = Math.toRadians(-17.728380);
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
    //SIM PID constants
    public static final double SIM_KP = 10;
    public static final double SIM_KI = 0.0;
    public static final double SIM_KD = 1;
     //SIM FEEDFORWARD constants
    public static final double SIM_KS = 0;
    public static final double SIM_KV = 0;
    //Physical Constants
    public static final GearBox GEARING = GearBox.fromReductionStages(5.0 * (22.0 / 16.0) * (47.0 / 18.0));
    public static final boolean INVERTED = false;
    public static final MotorMode MOTOR_MODE= MotorMode.BRAKE;
    public static final double MAXVEL = 10;
    public static final double ACELERATION = 5;
    //LIMITS
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
    public static final Time RAMP_RATE = Seconds.of(0.2); 
    public static final Distance MID_POINT = Millimeters.of(150);
    public static final Distance ARM_LENGTH = Meters.of(0.56);
    //TOLERANCE
    public static final Distance ARM_TOLERANCE = Millimeters.of(1); //TODO: TUNE
    //Telematry
    public static final TelemetryVerbosity MOTOR_TELEMATRY_MODE = TelemetryVerbosity.HIGH;
    public static final TelemetryVerbosity ARM_TELEMETRY_MODE = TelemetryVerbosity.HIGH;
}
