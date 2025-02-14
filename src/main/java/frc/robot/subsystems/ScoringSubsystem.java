// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
  /** Creates a new CoralItake. */
  // private final TalonFX left_coral = new TalonFX(4);
  private boolean intakingAlgae = false;

  private final TalonFX scoreMotor = new TalonFX(SCORE_MOTOR_PORT);

  public ScoringSubsystem() {}

  public Command DefaultCommand() {
    return run(
        () -> {
          // left_coral.stopMotor();
          scoreMotor.stopMotor();
        });
  }

  public Command setSpeed(double speed) {
    return run(
        () -> {
          // left_coral.set(speed);
          scoreMotor.set(speed);
        });
  }

  public Command algae(double speed) {
    return run(
        () -> {
          // left_coral.set(speed);
          scoreMotor.set(speed);
          intakingAlgae = true;
        });
  }

  public Command coral(double speed) {
    return run(
        () -> {
          // left_coral.set(speed);
          scoreMotor.set(speed);
          intakingAlgae = true;
        });
  }

  public boolean getCurrentSpike() {
    return scoreMotor.getStatorCurrent().getValueAsDouble() > 100;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
