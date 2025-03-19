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

import org.littletonrobotics.junction.Logger;

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

public class ExtensionSubsystem extends SubsystemBase {
  private LoggedTunableNumber tuneablePosition;
  private final TalonFX extensionMotorA = new TalonFX(EXTENSION_MOTOR_PORT_A);
  private final TalonFX extensionMotorB = new TalonFX(EXTENSION_MOTOR_PORT_B);
  private double elevatorSetpoint;
  private final String name = "Extension/";

  final MotionMagicVoltage m_lRequest;

  public ExtensionSubsystem() {
    // in init function
    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.75;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 100;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 100;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.MotorOutput.Inverted = extensionConfig.MotorOutput.Inverted;

    var extensionSlot0Configs = extensionConfig.Slot0;

    // set slot 0 gains
    extensionSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    extensionSlot0Configs.kS =
        EXTENSION_KS; // Add EXTENSION_KS V output to overcome static friction
    extensionSlot0Configs.kG = EXTENSION_KG;
    extensionSlot0Configs.kP =
        EXTENSION_KP; // A position error of EXTENSION_KP rotations results in 12 V output

    extensionSlot0Configs.kI = 0.0; // no output for integrated error
    extensionSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.0 V output

    // set Motion Magic settings
    var extensionMMConfig = extensionConfig.MotionMagic;
    extensionMMConfig.MotionMagicCruiseVelocity =
        EXTENSION_CRUISE_VELOCITY; // Target cruise velocity of EXTENSION_CRUISE_VELOCITY rps
    extensionMMConfig.MotionMagicAcceleration =
        EXTENSION_ACCELERATION; // Target acceleration of EXTENSION_ACCELERATION rps/s
    extensionMMConfig.MotionMagicJerk = EXTENSION_JERK; // Target jerk of EXTENSION_JERK rps/s/s

    // in init function
    tryUntilOk(5, () -> extensionMotorA.getConfigurator().apply(extensionConfig, 0.25));
    tryUntilOk(5, () -> extensionMotorB.getConfigurator().apply(extensionConfig, 0.25));

    extensionMotorB.setControl(new Follower(EXTENSION_MOTOR_PORT_A, false));
    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);

    tuneablePosition = new LoggedTunableNumber(name + "desiredPos", 0.0);
  }

  @Override
  public void periodic() {
    // System.out.println(tunableHeight.get());
    Logger.recordOutput(name + "setpoint", extensionMotorA.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput(name + "position", extensionMotorA.getPosition().getValueAsDouble());
    Logger.recordOutput(name + "velocity", extensionMotorA.getVelocity().getValueAsDouble());
    Logger.recordOutput(name + "acceleration", extensionMotorA.getAcceleration().getValueAsDouble());
    Logger.recordOutput(name + "duty cycle", extensionMotorA.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(name + "voltage", extensionMotorA.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(name + "PID Reference", extensionMotorA.getClosedLoopOutput().getValueAsDouble());
    Logger.recordOutput(name + "temperature ºC", extensionMotorA.getDeviceTemp().getValueAsDouble());

  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          // setSetpoint(tunableHeight.get());
          setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    extensionMotorA.setControl(
        m_lRequest
            .withPosition(Constants.tuningMode ? tuneablePosition.get() : setpoint)
            .withSlot(0));
  }

  public double getPosition() {
    return extensionMotorA.getPosition().getValueAsDouble();
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
    return Math.abs(extensionMotorA.getPosition().getValueAsDouble() - elevatorSetpoint) <= 0.10;
  }
}
