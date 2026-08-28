// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.hood.HoodConstants.HIGH_LIMIT;
import static frc.robot.subsystems.hood.HoodConstants.LOW_LIMIT;
import static frc.robot.subsystems.hood.HoodConstants.STARTING_POSITION;

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
import yams.mechanisms.positional.Arm;
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
            .withSimClosedLoopController(SIM_KP,SIM_KI,SIM_KD)
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
            .withSoftLimits(LOW_LIMIT, HIGH_LIMIT)
            .withStartingPosition(STARTING_POSITION);

    private final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getFalcon500(1), hoodMotorConfig);

    private final ArmConfig hoodConfig = new ArmConfig()
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withLength(Meters.of(0.3)) // Hood arm length for simulation
            .withHardLimits(Degrees.of(0), Degrees.of(120)); // The Hood can be modeled as an arm since it has a
                                                            // gravitational force acted upon based on the angle its in

    private final Arm hood = new Arm(hoodConfig, hoodSMC);

    public Hood() {
    }

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

  public void setAngleDirect(Angle angle)
  {
    hoodSMC.setPosition(angle);
  }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return hood.set(dutyCycleSupplier);
    }

    public Command setDutyCycle(double dutyCycle) {
        return hood.set(dutyCycle);
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