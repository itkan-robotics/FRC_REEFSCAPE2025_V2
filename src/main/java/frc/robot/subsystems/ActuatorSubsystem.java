// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ActuatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class ActuatorSubsystem extends SubsystemBase {

  private final TalonFX leftActuatorMotor = new TalonFX(LEFT_ACTUATOR_MOTOR_PORT, "static");
  private final TalonFX rightActuatorMotor = new TalonFX(RIGHT_ACTUATOR_MOTOR_PORT, "static");
  final MotionMagicVoltage m_lRequest;
  private LoggedTunableNumber tunableAngle;
  private double actuatorSetpoint;

  public ActuatorSubsystem() {
    // in init function
    var actuatorConfig = new TalonFXConfiguration();

    actuatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    actuatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    actuatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    actuatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 53;
    actuatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;
    actuatorConfig.CurrentLimits.SupplyCurrentLimit = 50;
    actuatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    actuatorConfig.CurrentLimits.StatorCurrentLimit = 50;
    actuatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    actuatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // set slot 0 gains
    var actuatorSlot0Configs = actuatorConfig.Slot0;
    actuatorSlot0Configs.kS = ACTUATOR_KS; // Add PIVOT_KS V output to overcome static friction
    actuatorSlot0Configs.kP =
        ACTUATOR_KP; // A position error of PIVOT_KP rotations results in 12 V output

    actuatorSlot0Configs.kI = 0.0; // no output for integrated error
    actuatorSlot0Configs.kD = 0.0; // no output for derived error

    // set Motion Magic settings
    var motionMagicConfigs = actuatorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        ACTUATOR_CRUISE_VELOCITY; // Target cruise velocity of PIVOT_CRUISE_VELOCITY rps
    motionMagicConfigs.MotionMagicAcceleration =
        ACTUATOR_ACCELERATION; // Target acceleration of PIVOT_ACCELERATION rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0; // No target jerk

    // in init function
    tryUntilOk(5, () -> rightActuatorMotor.getConfigurator().apply(actuatorConfig, 0.25));
    tryUntilOk(5, () -> leftActuatorMotor.getConfigurator().apply(actuatorConfig, 0.25));

    // leftPivotMotor.setControl(new Follower(rightPivotMotor.getDeviceID(), false));

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);
    tunableAngle = new LoggedTunableNumber("actuatorDesiredPos", 15);
  }

  @Override
  public void periodic() {}

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          setSetpoint(tunableAngle.get());
          // setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    actuatorSetpoint = setpoint;
    rightActuatorMotor.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
    leftActuatorMotor.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
    // rightActuatorMotor.setControl(m_lRequest.withPosition(tunableAngle.get()).withSlot(0));
    // leftActuatorMotor.setControl(m_lRequest.withPosition(tunableAngle.get()).withSlot(0));
  }

  public double getRightPosition() {
    return rightActuatorMotor.getPosition().getValueAsDouble();
  }

  public double getLeftPosition() {
    return leftActuatorMotor.getPosition().getValueAsDouble();
  }

  public double getPositionRequest() {
    return actuatorSetpoint;
  }

  public boolean setpointReached() {
    // SmartDashboard.putNumber(
    //     "/SetPointReached/Actuator",
    //     Math.abs((getRightPosition() + getLeftPosition()) / 2 - actuatorSetpoint));
    return Math.abs((getRightPosition() + getLeftPosition()) / 2 - actuatorSetpoint) <= 0.20;
  }
}
