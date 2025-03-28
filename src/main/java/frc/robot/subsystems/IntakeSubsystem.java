// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake_Motor_Port;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeMotor = new TalonFX(Intake_Motor_Port);

  public IntakeSubsystem() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          intakeMotor.set(-0.08);
        });
  }

  public Command setIntakeSpeed(double speed) {
    return run(
        () -> {
          intakeMotor.set(speed * -1);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
