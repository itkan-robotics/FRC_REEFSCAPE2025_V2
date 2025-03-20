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

import static frc.robot.Constants.ArmConstants.ShoulderConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggingUtil;

public class ShoulderSubsystem extends SubsystemBase {
  private LoggedTunableNumber tuneablePosition;
  private final TalonFX leftShoulderMotor = new TalonFX(LEFT_SHOULDER_MOTOR_PORT);
  private final TalonFX rightShoulderMotor = new TalonFX(SHOULDER_MOTOR_PORT);
  private double shoulderSetpoint;
  private final String name = "Shoulder";

  final MotionMagicVoltage m_lRequest;

  public ShoulderSubsystem() {
    // in init function
    var shoulderConfig = new TalonFXConfiguration();
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
    // shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.75;
    shoulderConfig.CurrentLimits.SupplyCurrentLimit = 25;
    shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shoulderConfig.CurrentLimits.StatorCurrentLimit = 25;
    shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shoulderConfig.MotorOutput.Inverted = shoulderConfig.MotorOutput.Inverted;

    var shoulderSlot0Configs = shoulderConfig.Slot0;

    // set slot 0 gains
    shoulderSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    // shoulderSlot0Configs.kS = SHOULDER_KS; // Add SHOULDER_KS V output to overcome static
    // friction
    shoulderSlot0Configs.kG = SHOULDER_KG;
    shoulderSlot0Configs.kV = SHOULDER_KV;
    shoulderSlot0Configs.kP =
        SHOULDER_KP; // A position error of SHOULDER_KP rotations results in 12 V output

    // set Motion Magic settings
    var extensionMMConfig = shoulderConfig.MotionMagic;
    extensionMMConfig.MotionMagicCruiseVelocity =
        SHOULDER_CRUISE_VELOCITY; // Target cruise velocity of SHOULDER_CRUISE_VELOCITY rps
    extensionMMConfig.MotionMagicAcceleration =
        SHOULDER_ACCELERATION; // Target acceleration of SHOULDER_ACCELERATION rps/s

    // shoulderSlot0Configs.kI = 0.0; // no output for integrated error
    // shoulderSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.0 V output

    // extensionMMConfig.MotionMagicJerk = SHOULDER_JERK; // Target jerk of SHOULDER_JERK rps/s/s

    // in init function
    tryUntilOk(5, () -> leftShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));
    tryUntilOk(5, () -> rightShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));

    rightShoulderMotor.setControl(new Follower(LEFT_SHOULDER_MOTOR_PORT, true));

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);

    tuneablePosition = new LoggedTunableNumber(name + "/DesiredPos", 0.0);
  }

  @Override
  public void periodic() {
    LoggingUtil.logMotor(name, leftShoulderMotor);
  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    leftShoulderMotor.setControl(
        m_lRequest
            .withPosition(Constants.tuningMode ? tuneablePosition.get() : setpoint)
            .withSlot(0));
  }

  public double getPosition() {
    return leftShoulderMotor.getPosition().getValueAsDouble();
  }

  public double getPositionRequest() {
    return shoulderSetpoint;
  }
}
