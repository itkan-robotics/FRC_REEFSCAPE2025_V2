// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX leftPivotMotor = new TalonFX(LEFT_PIVOT_MOTOR_PORT);
  private final TalonFX rightPivotMotor = new TalonFX(RIGHT_PIVOT_MOTOR_PORT);
  final MotionMagicVoltage m_lRequest;

  public PivotSubsystem() {

    // in init function
    // Target jerk of 4000 rps/s/s (0.1 seconds)
    // in init function
    var leftConfig = new TalonFXConfiguration();
    var rightConfig = new TalonFXConfiguration();

    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 75;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // set slot 0 gains
    var leftSlot0Configs = leftConfig.Slot0;
    leftSlot0Configs.kS = PIVOT_KS; // Add 0.25 V output to overcome static friction
    leftSlot0Configs.kP = PIVOT_KP; // A position error of 2.5 rotations results in 12 V output

    leftSlot0Configs.kI = 0.0; // no output for integrated error
    leftSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = leftConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        PIVOT_CRUISE_VELOCITY; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        PIVOT_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // in init function
    tryUntilOk(5, () -> rightPivotMotor.getConfigurator().apply(leftConfig, 0.25));
    tryUntilOk(5, () -> leftPivotMotor.getConfigurator().apply(leftConfig, 0.25));

    leftPivotMotor.setControl(new Follower(rightPivotMotor.getDeviceID(), false));

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

  public Command resetPivot() {
    return run(
        () -> {
          resetPosition();
        });
  }

  public void setSetpoint(double setpoint) {
    rightPivotMotor.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
  }

  public double getRightPosition() {
    return rightPivotMotor.getPosition().getValueAsDouble();
  }

  public void resetPosition() {
    setSetpoint(5);
    // rightMotor.setPosition(0.0, 0.25);
  }
}
