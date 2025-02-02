// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralOuttake. */
  private final TalonFX coralIntakeMotor = new TalonFX(INTAKE_MOTOR_PORT);

  public IntakeSubsystem() {}

  public Command DefaultCommand() {
    return run(
        () -> {
          coralIntakeMotor.stopMotor();
        });
  }

  public Command setSpeed(double speed) {
    return run(
        () -> {
          coralIntakeMotor.set(speed);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
