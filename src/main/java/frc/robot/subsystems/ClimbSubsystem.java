// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_PORT);

  public ClimbSubsystem() {
    // in init function
    var climbConfig = new TalonFXConfiguration();
    climbConfig.CurrentLimits.SupplyCurrentLimit = 40;
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.CurrentLimits.StatorCurrentLimit = 40;
    climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> climbMotor.getConfigurator().apply(climbConfig, 0.25));
  }

  public Command setSpeed(double speed) {
    return Commands.run(
        () -> {
          climbMotor.set(speed);
        });
  }

  @Override
  public void periodic() {
    // climbServo.set(tunableAngle.get());
    // This method will be called once per scheduler run
  }
}
