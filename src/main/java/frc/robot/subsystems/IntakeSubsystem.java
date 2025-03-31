// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake_Motor_Port;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeMotor = new TalonFX(Intake_Motor_Port);

  private Timer timer = new Timer();

  public IntakeSubsystem() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    timer.start();
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          intakeMotor.set(-0.0);
          // System.out.println(timer.get());
        });
  }

  public Command SmartIntake() {
    return run(
        () -> {
          if (intakeMotor.getMotorVoltage().getValueAsDouble() < 10.5) {
            intakeMotor.set(-0.2);
          } else {
            intakeMotor.set(1);
          }
        });
  }

  public Command setIntakeSpeed(double speed) {
    return run(
        () -> {
          intakeMotor.set(-speed);
        });
  }

  public Command setIntakeTimed() {
    return run(
        () -> {
          intakeTimed(1, 0.25);
        });
  }

  public Command restartTime() {
    return run(
        () -> {
          timer.restart();
        });
  }

  public void intakeTimed(double speed, double time) {
    if (timer.get() < time) {
      intakeMotor.set(speed);
    } else {
      intakeMotor.set(0);
    }
    System.out.println(timer.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
