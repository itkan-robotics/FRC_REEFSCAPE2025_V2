// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttake extends SubsystemBase {
  /** Creates a new CoralOuttake. */
  private final TalonFX coral_outtake = new TalonFX(CORAL_OUTTAKE_MOTOR_PORT);

  public CoralOuttake() {

    var talonFXConfigurator = coral_outtake.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    // set invert to CW+ and apply config change
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          coral_outtake.stopMotor();
        });
  }

  public Command Outtake(double speed) {
    return run(
        () -> {
          coral_outtake.set(speed);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

