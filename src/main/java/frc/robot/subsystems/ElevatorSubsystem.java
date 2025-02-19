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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorSubsystem extends SubsystemBase {
  private LoggedTunableNumber tunableAngle;
  private final TalonFX elevator = new TalonFX(ELEVATOR_MOTOR_PORT);
  // private final TalonFX rightMotor = new TalonFX(RIGHT_ELEVATOR_MOTOR_PORT);
  // private final Follower rightMotorFollower = new Follower(LEFT_ELEVATOR_MOTOR_PORT,true);

  final MotionMagicVoltage m_lRequest;

  public ElevatorSubsystem() {

    // in init function
    var elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    var elevatorSlot0Configs = elevatorConfig.Slot0;

    // set slot 0 gains
    elevatorSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    elevatorSlot0Configs.kS = ELEVATOR_KS; // Add ELEVATOR_KS V output to overcome static friction
    elevatorSlot0Configs.kG = ELEVATOR_KG;
    elevatorSlot0Configs.kP =
        ELEVATOR_KP; // A position error of ELEVATOR_KP rotations results in 12 V output

    elevatorSlot0Configs.kI = 0.0; // no output for integrated error
    elevatorSlot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = elevatorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        ELEVATOR_CRUISE_VELOCITY; // Target cruise velocity of ELEVATOR_CRUISE_VELOCITY rps
    motionMagicConfigs.MotionMagicAcceleration =
        ELEVATOR_ACCELERATION; // Target acceleration of ELEVATOR_ACCELERATION rps/s
    motionMagicConfigs.MotionMagicJerk = ELEVATOR_JERK; // Target jerk of ELEVATOR_JERK rps/s/s

    // in init function
    tryUntilOk(5, () -> elevator.getConfigurator().apply(elevatorConfig, 0.25));

    // rightMotor.setControl(rightMotorFollower);

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);
    tunableAngle = new LoggedTunableNumber("elevatorRotation1234", 5);
  }

  @Override
  public void periodic() {
    //System.out.println(tunableAngle.get());
  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          //setSetpoint(tunableAngle.get());
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

  public double getPosition() {
    return elevator.getPosition().getValueAsDouble();
  }

  public double getSlowDownMult() {
    if (getPosition() > 10) {
      return 0.5;
    } else {
      return 1.0;
    }
  }

  public void resetPosition() {
    setSetpoint(0); // rightMotor.setPosition(0.0, 0.25);
  }
}
