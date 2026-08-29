package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ShooterConstants {
    //phisycal info
        public static final int MOTOR_ID = 0;
        public static final double GEARING = 3;
        public static final boolean INVERTED = false;
        public static final MotorMode NEUTRAL_MODE = MotorMode.COAST;
    //control loop
        public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
        public static final AngularVelocity FEED_SPEED = RPM.of(720);
        public static final AngularVelocity UNFEED_SPEED = RPM.of(-720);
        public static final Voltage FEED_VOLTAGE = Volts.of(12);
        public static final Voltage UNFEED_VOLTAGE = Volts.of(-12);
        // REAL PID
        public static final double REAL_KP = 50;
        public static final double REAL_KI = 0;
        public static final double REAL_KD = 0;
        // REAL FEEDFOWARD
        public static final double REAL_KS = 0;
        public static final double REAL_KV = 0;
        public static final double REAL_KA = 0;

        // SIM PID
        public static final double SIM_KP = 1;
        public static final double SIM_KI = 0;
        public static final double SIM_KD = 0;
        // SIM FEEDFOWARD
        public static final double SIM_KS = 0;
        public static final double SIM_KV = 0;
        public static final double SIM_KA = 0;
    //limits
        public static Current STATOR_LIMIT = Amps.of(120);
        public static Current SUPPLY_LIMIT = Amps.of(60);

    //general
        public static final TelemetryVerbosity MOTOR_VERBOSITY = TelemetryVerbosity.HIGH;
        public static final TelemetryVerbosity MECHANISM_VERBOSITY = TelemetryVerbosity.HIGH;

}