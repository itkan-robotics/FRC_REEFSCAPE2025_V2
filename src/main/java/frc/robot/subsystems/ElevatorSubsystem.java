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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorSubsystem extends SubsystemBase {
  private LoggedTunableNumber tunableHeight;
  private final TalonFX leftElevatorMotor = new TalonFX(ELEVATOR_MOTOR_PORT_LEFT);
  private final TalonFX rightElevatorMotor = new TalonFX(ELEVATOR_MOTOR_PORT_RIGHT);
  private double elevatorSetpoint;
  // private final TalonFX rightMotor = new TalonFX(RIGHT_ELEVATOR_MOTOR_PORT);
  // private final Follower rightMotorFollower = new Follower(LEFT_ELEVATOR_MOTOR_PORT,true);

  final MotionMagicVoltage m_lRequest;

  public ElevatorSubsystem() {
    // in init function
    var leftElevatorConfig = new TalonFXConfiguration();
    leftElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
    leftElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    leftElevatorConfig.CurrentLimits.SupplyCurrentLimit = 100;
    leftElevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftElevatorConfig.CurrentLimits.StatorCurrentLimit = 100;
    leftElevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    var leftElevatorSlot0Configs = leftElevatorConfig.Slot0;

    // set slot 0 gains
    leftElevatorSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    leftElevatorSlot0Configs.kS =
        ELEVATOR_KS; // Add ELEVATOR_KS V output to overcome static friction
    leftElevatorSlot0Configs.kG = ELEVATOR_KG;
    leftElevatorSlot0Configs.kP =
        ELEVATOR_KP; // A position error of ELEVATOR_KP rotations results in 12 V output

    leftElevatorSlot0Configs.kI = 0.0; // no output for integrated error
    leftElevatorSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.0 V output

    // set Motion Magic settings
    var leftMotionMagicConfigs = leftElevatorConfig.MotionMagic;
    leftMotionMagicConfigs.MotionMagicCruiseVelocity =
        ELEVATOR_CRUISE_VELOCITY; // Target cruise velocity of ELEVATOR_CRUISE_VELOCITY rps
    leftMotionMagicConfigs.MotionMagicAcceleration =
        ELEVATOR_ACCELERATION; // Target acceleration of ELEVATOR_ACCELERATION rps/s
    leftMotionMagicConfigs.MotionMagicJerk = ELEVATOR_JERK; // Target jerk of ELEVATOR_JERK rps/s/s

    var rightElevatorConfig = new TalonFXConfiguration();
    rightElevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rightElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rightElevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
    rightElevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    rightElevatorConfig.CurrentLimits.SupplyCurrentLimit = 120;
    rightElevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightElevatorConfig.CurrentLimits.StatorCurrentLimit = 120;
    rightElevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightElevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var rightElevatorSlot0Configs = rightElevatorConfig.Slot0;

    // set slot 0 gains
    rightElevatorSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    rightElevatorSlot0Configs.kS =
        ELEVATOR_KS; // Add ELEVATOR_KS V output to overcome static friction
    rightElevatorSlot0Configs.kG = ELEVATOR_KG;
    rightElevatorSlot0Configs.kP =
        ELEVATOR_KP; // A position error of ELEVATOR_KP rotations results in 12 V output

    rightElevatorSlot0Configs.kI = 0.0; // no output for integrated error
    rightElevatorSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.0 V output

    // set Motion Magic settings
    var rightMotionMagicConfigs = rightElevatorConfig.MotionMagic;
    rightMotionMagicConfigs.MotionMagicCruiseVelocity =
        ELEVATOR_CRUISE_VELOCITY; // Target cruise velocity of ELEVATOR_CRUISE_VELOCITY rps
    rightMotionMagicConfigs.MotionMagicAcceleration =
        ELEVATOR_ACCELERATION; // Target acceleration of ELEVATOR_ACCELERATION rps/s
    rightMotionMagicConfigs.MotionMagicJerk = ELEVATOR_JERK; // Target jerk of ELEVATOR_JERK rps/s/s

    // in init function
    tryUntilOk(5, () -> leftElevatorMotor.getConfigurator().apply(leftElevatorConfig, 0.25));
    // tryUntilOk(5, () -> rightElevatorMotor.getConfigurator().apply(rightElevatorConfig, 0.25));

    // rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);
    tunableHeight = new LoggedTunableNumber("elevatorDesiredPos", 24.75);
  }

  @Override
  public void periodic() {
    // System.out.println(tunableHeight.get());
  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          // setSetpoint(tunableHeight.get());
          setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    // leftElevatorMotor.setControl(m_lRequest.withPosition(tunableHeight.get()).withSlot(0));
    leftElevatorMotor.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
  }

  public double getPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  public double getPositionRequest() {
    return elevatorSetpoint;
  }

  public double getSlowDownMult() {
    if (getPosition() > 10) {
      return 0.6;
    } else {
      return 0.85;
    }
  }

  public boolean setpointReached() {
    // SmartDashboard.putNumber(
    //     "/SetPointReached/Elevator",
    //     Math.abs(leftElevatorMotor.getPosition().getValueAsDouble() - elevatorSetpoint));
    return Math.abs(leftElevatorMotor.getPosition().getValueAsDouble() - elevatorSetpoint) <= 0.10;
  }
}
