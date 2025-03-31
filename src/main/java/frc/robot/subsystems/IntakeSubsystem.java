// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake_Motor_Port;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeMotor = new TalonFX(Intake_Motor_Port);

  DigitalInput ranger = new DigitalInput(0);

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
          intakeMotor.set(-0.045);
        });
  }

  public Command setIntakeSpeed(double speed) {
    return run(
        () -> {
          intakeMotor.set(-speed);
        });
  }

  public void setIntake(double speed) {
    intakeMotor.set(-speed);
  }

  public boolean isIntaked() {
    return !ranger.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("ranger/Ranger Value", ranger.get());
  }
}
