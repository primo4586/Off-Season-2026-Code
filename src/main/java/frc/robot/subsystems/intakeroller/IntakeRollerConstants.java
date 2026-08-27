package frc.robot.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class IntakeRollerConstants {
    //control loop
        public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;

        // REAL PID
        public static final float REAL_KP = 0;
        public static final float REAL_KI = 0;
        public static final float REAL_KD = 0;
        // REAL FEEDFOWARD
        public static final float REAL_KS = 0;
        public static final float REAL_KV = 0;
        public static final float REAL_KA = 0;

        // SIM PID
        public static final float SIM_KP = 0;
        public static final float SIM_KI = 0;
        public static final float SIM_KD = 0;
        // SIM FEEDFOWARD
        public static final float SIM_KS = 0;
        public static final float SIM_KV = 0;
        public static final float SIM_KA = 0;
    //phisycal info
        public static final double GEARING = 3;
        public static final boolean INVERTED = false;
        public static final MotorMode NEUTRAL_MODE = MotorMode.COAST;
    //limits
        public static Current STATOR_LIMIT = Amps.of(100);
        public static Current SUPPLY_LIMIT = Amps.of(50);

    //general
        public static final TelemetryVerbosity VERBOSITY = TelemetryVerbosity.HIGH;

}
