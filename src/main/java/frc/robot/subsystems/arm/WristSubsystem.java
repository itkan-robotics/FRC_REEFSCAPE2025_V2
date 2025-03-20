// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.WristConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  private LoggedTunableNumber tuneablePosition;
  private final TalonFX wristMotor = new TalonFX(WRIST_MOTOR_PORT_A);
  private double shoulderSetpoint;
  private final String name = "Wrist/";

  final MotionMagicVoltage m_lRequest;

  public WristSubsystem() {
    // in init function
    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.75;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 30;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = 30;
    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = wristConfig.MotorOutput.Inverted;

    var shoulderSlot0Configs = wristConfig.Slot0;

    // set slot 0 gains
    shoulderSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    // shoulderSlot0Configs.kS = WRIST_KS; // Add WRIST_KS V output to overcome static friction
    shoulderSlot0Configs.kG = WRIST_KG;
    shoulderSlot0Configs.kV = WRIST_KV;
    shoulderSlot0Configs.kP =
        WRIST_KP; // A position error of WRIST_KP rotations results in 12 V output

    // set Motion Magic settings
    var extensionMMConfig = wristConfig.MotionMagic;
    extensionMMConfig.MotionMagicCruiseVelocity =
        WRIST_CRUISE_VELOCITY; // Target cruise velocity of WRIST_CRUISE_VELOCITY rps
    extensionMMConfig.MotionMagicAcceleration =
    WRIST_CRUISE_VELOCITY / 0.5; // Reach target cruise velocity in 0.5 s

    // shoulderSlot0Configs.kI = 0.0; // no output for integrated error
    // shoulderSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.0 V output

    
    // extensionMMConfig.MotionMagicJerk = WRIST_JERK; // Target jerk of WRIST_JERK rps/s/s

    // in init function
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);

    tuneablePosition = new LoggedTunableNumber(name + "desiredPos", 0.0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(name + "setpoint", wristMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput(name + "position", wristMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(name + "velocity", wristMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(name + "acceleration", wristMotor.getAcceleration().getValueAsDouble());
    Logger.recordOutput(name + "duty cycle", wristMotor.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(name + "voltage", wristMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        name + "PID Reference", wristMotor.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput(name + "temperature ºC", wristMotor.getDeviceTemp().getValueAsDouble());
  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    wristMotor.setControl(
        m_lRequest
            .withPosition(Constants.tuningMode ? tuneablePosition.get() : setpoint)
            .withSlot(0));
  }

  public double getPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public double getPositionRequest() {
    return shoulderSetpoint;
  }
}
