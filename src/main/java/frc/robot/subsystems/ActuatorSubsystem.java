// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PivotConstants.*;
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

  private final TalonFX leftPivotMotor = new TalonFX(LEFT_ACTUATOR_MOTOR_PORT, "static");
  private final TalonFX rightPivotMotor = new TalonFX(RIGHT_ACTUATOR_MOTOR_PORT, "static");
  final MotionMagicVoltage m_lRequest;
  private LoggedTunableNumber tunableAngle;

  public ActuatorSubsystem() {
    // in init function
    var actuatorConfig = new TalonFXConfiguration();

    actuatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    actuatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    actuatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    actuatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 53;
    actuatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;
    actuatorConfig.CurrentLimits.SupplyCurrentLimit = 100;
    actuatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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
    tryUntilOk(5, () -> rightPivotMotor.getConfigurator().apply(actuatorConfig, 0.25));
    tryUntilOk(5, () -> leftPivotMotor.getConfigurator().apply(actuatorConfig, 0.25));

    // leftPivotMotor.setControl(new Follower(rightPivotMotor.getDeviceID(), false));

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);
    tunableAngle = new LoggedTunableNumber("actuator123", 15);
  }

  @Override
  public void periodic() {}

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          //setSetpoint(tunableAngle.get());
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
    leftPivotMotor.setControl(m_lRequest.withPosition(setpoint).withSlot(0));
  }

  public double getRightPosition() {
    return rightPivotMotor.getPosition().getValueAsDouble();
  }

  public double getLeftPosition() {
    return leftPivotMotor.getPosition().getValueAsDouble();
  }

  public void resetPosition() {
    setSetpoint(5.0);
  }
}
