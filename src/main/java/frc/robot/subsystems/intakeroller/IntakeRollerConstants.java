package frc.robot.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class IntakeRollerConstants {
    //control loop
        public static final ControlMode CONTROL_MODE = ControlMode.CLOSED_LOOP;
        public static final AngularVelocity INTAKE_SPEED = RPM.of(120);
        public static final AngularVelocity OUTTAKE_SPEED = RPM.of(-120);
        // REAL PID
        public static final float REAL_KP = 50;
        public static final float REAL_KI = 0;
        public static final float REAL_KD = 0;
        // REAL FEEDFOWARD
        public static final float REAL_KS = 0;
        public static final float REAL_KV = 0;
        public static final float REAL_KA = 0;

        // SIM PID
        public static final float SIM_KP = 1;
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
        public static final int MOTOR_ID = 0;
    //limits
        public static Current STATOR_LIMIT = Amps.of(100);
        public static Current SUPPLY_LIMIT = Amps.of(50);

    //general
        public static final TelemetryVerbosity MOTOR_VERBOSITY = TelemetryVerbosity.HIGH;
        public static final TelemetryVerbosity MECHANISM_VERBOSITY = TelemetryVerbosity.HIGH;

}