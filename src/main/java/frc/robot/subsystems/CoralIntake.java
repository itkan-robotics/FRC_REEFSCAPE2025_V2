// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.LEFT_CORAL_INATKE_MOTOR_PORT;
import static frc.robot.Constants.ElevatorConstants.RIGHT_CORAL_INATKE_MOTOR_PORT;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralItake. */
  private final TalonFX left_coral = new TalonFX(LEFT_CORAL_INATKE_MOTOR_PORT);

  private final TalonFX right_coral = new TalonFX(RIGHT_CORAL_INATKE_MOTOR_PORT);

  public CoralIntake() {

    var talonFXConfigurator = left_coral.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    // set invert to CW+ and apply config change
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          left_coral.stopMotor();
          right_coral.stopMotor();
        });
  }

  public Command Intake(double speed) {
    return run(
        () -> {
          left_coral.set(speed);
          right_coral.set(speed);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
