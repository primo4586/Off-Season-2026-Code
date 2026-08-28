package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class HoodConstants {
    //control loop
        public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;

        //TRAPEZOIDAL PROFILE
        public static final AngularVelocity MAX_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(2500);

        // REAL PID
        public static final double REAL_KP = 50;
        public static final double REAL_KI = 0;
        public static final double REAL_KD = 0;
        // REAL FEEDFOWARD
        public static final double REAL_KS = 0;
        public static final double REAL_KG = 0;
        public static final double REAL_KV = 0;

        // SIM PID
        public static final double SIM_KP = 10;
        public static final double SIM_KI = 0;
        public static final double SIM_KD = 0;
        // SIM FEEDFOWARD
        public static final double SIM_KS = 0;
        public static final double SIM_KG = 0;
        public static final double SIM_KV = 0;
    //phisycal info
        public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
        public static final boolean INVERTED = false;
        public static final MotorMode NEUTRAL_MODE = MotorMode.COAST;
        public static final int MOTOR_ID = 0;
        public static final Distance LENGTH_OF_SIM_ARM = Millimeter.of(20);
        public static final Angle STARTING_POSITION = Degrees.of(0);
        public static final Angle HIGH_LIMIT = Degrees.of(100);
        public static final Angle LOW_LIMIT = Degrees.of(0);
        public static final Angle SIM_HARD_HIGH_LIMIT = Degrees.of(100);
        public static final Angle SIM_HARD_LOW_LIMIT = Degrees.of(0);
    //limits
        public static Current STATOR_LIMIT = Amps.of(100);
        public static Current SUPPLY_LIMIT = Amps.of(50);
        public static Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
    //general
        public static final TelemetryVerbosity MOTOR_VERBOSITY = TelemetryVerbosity.HIGH;
        public static final TelemetryVerbosity MECHANISM_VERBOSITY = TelemetryVerbosity.HIGH;
        public static final Angle TOLERANCE = Degrees.of(0);

}