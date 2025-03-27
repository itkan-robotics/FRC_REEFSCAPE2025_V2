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

import static frc.robot.Constants.ArmConstants.ExtensionConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggingUtil;

public class ExtensionSubsystem extends SubsystemBase {
  private LoggedTunableNumber tuneablePosition;
  private final TalonFX extensionMotorRight = new TalonFX(EXTENSION_MOTOR_PORT_RIGHT, "static");
  private final TalonFX extensionMotorLeft = new TalonFX(EXTENSION_MOTOR_PORT_LEFT, "static");
  private double elevatorSetpoint;
  private final String name = "Extension";

  final MotionMagicVoltage m_lRequest;

  public ExtensionSubsystem() {
    // in init function
    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_EXTENSION_POS;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_EXTENSION_POS;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 50;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 50;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var extensionSlot0Configs = extensionConfig.Slot0;

    // set slot 0 gains
    extensionSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    extensionSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    extensionSlot0Configs.kG = EXTENSION_KG;
    extensionSlot0Configs.kV = EXTENSION_KV;
    extensionSlot0Configs.kP =
        EXTENSION_KP; // A position error of EXTENSION_KP rotations results in 12 V output

    // set Motion Magic settings
    var extensionMMConfig = extensionConfig.MotionMagic;

    extensionMMConfig.MotionMagicCruiseVelocity = EXTENSION_CRUISE_VELOCITY;
    extensionMMConfig.MotionMagicAcceleration = EXTENSION_ACCELERATION;

    // in init function
    tryUntilOk(5, () -> extensionMotorRight.getConfigurator().apply(extensionConfig, 0.25));
    tryUntilOk(5, () -> extensionMotorLeft.getConfigurator().apply(extensionConfig, 0.25));

    extensionMotorLeft.setControl(new Follower(EXTENSION_MOTOR_PORT_RIGHT, true));
    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(2);

    tuneablePosition = new LoggedTunableNumber(name + "/DesiredPos", 0.0);
  }

  @Override
  public void periodic() {
    // System.out.println(tunableHeight.get());
    LoggingUtil.logMotor(name, extensionMotorRight);
  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          // setSetpoint(tunableHeight.get());
          setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    extensionMotorRight.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
  }

  public double getPosition() {
    return extensionMotorRight.getPosition().getValueAsDouble();
  }

  public double getPositionRequest() {
    return elevatorSetpoint;
  }

  public double getSlowDownMult() {
    return 1.0 - (getPosition() * 0.0125);
  }

  public boolean setpointReached() {
    // SmartDashboard.putNumber(
    //     "/SetPointReached/Elevator",
    //     Math.abs(leftElevatorMotor.getPosition().getValueAsDouble() - elevatorSetpoint));
    return Math.abs(extensionMotorRight.getPosition().getValueAsDouble() - elevatorSetpoint)
        <= 0.10;
  }
}
