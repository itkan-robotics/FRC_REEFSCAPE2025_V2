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

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevator = new TalonFX(LEFT_ELEVATOR_MOTOR_PORT);
  // private final TalonFX rightMotor = new TalonFX(RIGHT_ELEVATOR_MOTOR_PORT);
  // private final Follower rightMotorFollower = new Follower(LEFT_ELEVATOR_MOTOR_PORT,true);

  final MotionMagicVoltage m_lRequest;

  public ElevatorSubsystem() {
    // in init function
    // Target jerk of 4000 rps/s/s (0.1 seconds)
    // in init function
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 19;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.5;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 25;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.MotorOutput.Inverted =
        LEFT_ELEVATOR_IS_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // set slot 0 gains
    var leftSlot0Configs = leftConfig.Slot0;
    leftSlot0Configs.kS = 7.5; // Add 0.25 V output to overcome static friction
    leftSlot0Configs.kP = 50; // A position error of 2.5 rotations results in 12 V output

    leftSlot0Configs.kI = 0.0; // no output for integrated error
    leftSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = leftConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 60; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        250; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 750; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // in init function
    tryUntilOk(5, () -> elevator.getConfigurator().apply(leftConfig, 0.25));

    // rightMotor.setControl(rightMotorFollower);

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);
  }

  @Override
  public void periodic() {}

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          setSetpoint(setpoint);
        });
  }

  public Command resetElevators() {
    return run(
        () -> {
          resetPosition();
        });
  }

  public void setSetpoint(double setpoint) {
    elevator.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
  }

  public double getLeftPosition() {
    return elevator.getPosition().getValueAsDouble();
  }

  public void resetPosition() {
    setSetpoint(0); // rightMotor.setPosition(0.0, 0.25);
  }
}
