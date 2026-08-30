package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
public class ShooterConstants {
    //phisycal info
        public static final int MOTOR_ID = 30; // id for main motor; note: motor must be on canivore bus
        public static final int FOLLOWER_ID = 31; // id for follower motor; note: motor must be on canivore bus
        public static final double GEARING = 27/20;
        public static final boolean INVERTED = false;
        public static final boolean FOLLOWER_INVERTED = false;
        public static final MotorMode NEUTRAL_MODE = MotorMode.COAST;

    //control loop
        // general stuff
        public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
        public static final AngularVelocity SHOOT_SPEED = RotationsPerSecond.of(20);
        public static final AngularVelocity PASS_SPEED = RotationsPerSecond.of(20);
        public static final AngularVelocity REST_SPEED = RotationsPerSecond.of(4);

        // REAL PID
        public static final double REAL_KP = 0.2;
        public static final double REAL_KI = 0;
        public static final double REAL_KD = 0;
        // REAL FEEDFOWARD
        public static final double REAL_KS = 0.21606;
        public static final double REAL_KV = 0.12316;
        public static final double REAL_KA = 0.0080154;

        // SIM PID
        public static final double SIM_KP = 10;
        public static final double SIM_KI = 0;
        public static final double SIM_KD = 0.03;
        // SIM FEEDFOWARD
        public static final double SIM_KS = 0.17;
        public static final double SIM_KV = 0.16;
        public static final double SIM_KA = 0.02;
    //limits
        public static Current STATOR_LIMIT = Amps.of(200);
        public static Current SUPPLY_LIMIT = Amps.of(200);

    //general
        public static final TelemetryVerbosity MOTOR_VERBOSITY = TelemetryVerbosity.HIGH;
        public static final TelemetryVerbosity MECHANISM_VERBOSITY = TelemetryVerbosity.HIGH;

}