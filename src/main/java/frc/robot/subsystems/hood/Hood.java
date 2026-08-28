// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import static frc.robot.subsystems.hood.HoodConstants.*;;

public class Hood extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(2);

    private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(REAL_KP, REAL_KI, REAL_KD)
            .withSimClosedLoopController(SIM_KP, SIM_KI, SIM_KD)
            .withTrapezoidalProfile(MAX_VELOCITY, MAX_ACCELERATION)
            .withGearing(GEARING)
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(STATOR_LIMIT)
            .withMotorInverted(INVERTED)
            .withClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE)
            .withOpenLoopRampRate(OPEN_LOOP_RAMP_RATE)
            .withFeedforward(new ArmFeedforward(REAL_KS, REAL_KG, REAL_KV))
            .withSimFeedforward(new ArmFeedforward(REAL_KS, REAL_KG, REAL_KV))
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withStartingPosition(STARTING_POSITION);

    private final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getFalcon500(1),
            hoodMotorConfig);

    private final PivotConfig hoodConfig = new PivotConfig()
            .withTelemetry("Hood", TelemetryVerbosity.HIGH) // The Hood can be modeled as an arm since it has a
            .withHardLimits(Degrees.of(0), Degrees.of(90)); // gravitational force acted upon based on the angle its in

    private final Pivot hood = new Pivot(hoodConfig, hoodSMC);

    public Hood() {
    }

    /**
     * Run the arm to the given angle, does not stop when the arm reaches the
     * setpoint.
     * 
     * @param angle Angle to go to.
     * @return A command.
     */
    public Command run(Angle angle) {
        return hood.run(angle);
    }

    /**
     * Run the arm to the given angle, ends the command when the arm reaches the
     * setpoint within tolerance.
     * 
     * @param angle     Angle to go to.
     * @return A Command
     */
    public Command runTo(Angle angle) {
        return hood.runTo(angle, TOLERANCE);
    }

    /**
     * Set arm closed loop controller to go to the specified mechanism position.
     * 
     * @param angle Angle to go to.
     */
    public void setAngleSetpoint(Angle angle) {
        hood.setMechanismPositionSetpoint(angle);
    }

    /**
     * Move the arm up and down.
     * 
     * @param dutycycle [-1, 1] speed to set the arm too.
     */
    public Command set(double dutycycle) {
        return hood.set(dutycycle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}