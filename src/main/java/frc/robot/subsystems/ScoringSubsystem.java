// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
  /** Creates a new CoralItake. */
  // private final TalonFX left_coral = new TalonFX(4);
  private boolean intakingAlgae = false;

  private boolean outtake = false;

  private final TalonFX scoreMotor = new TalonFX(SCORE_MOTOR_PORT);

  public ScoringSubsystem() {
    // in init function
    var scoringConfig = new TalonFXConfiguration();
    scoringConfig.CurrentLimits.SupplyCurrentLimit = 40;
    scoringConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    scoringConfig.CurrentLimits.StatorCurrentLimit = 40;
    scoringConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> scoreMotor.getConfigurator().apply(scoringConfig, 0.25));
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          scoreMotor.set(0.005);
          // left_coral.stopMotor();
          // if (intakingAlgae) {
          //   scoreMotor.set(0.15);
          // } else if (outtake) {
          //   scoreMotor.set(-0.05);
          // }
        });
  }

  /**
   * Sets the speed of the outtake. A positive value means the coral outtakes/algae intakes, while a
   * negative value means the coral doesn't outtake / algae outtakes
   *
   * @param speed values explained in description
   */
  public Command setSpeed(double speed) {
    return run(
        () -> {
          scoreMotor.set(speed);
          outtake = true;
        });
  }

  public Command intakeAlgae(double speed) {
    return run(
        () -> {
          scoreMotor.set(getCurrentSpike() ? 0.05 : speed);
          intakingAlgae = true;
        });
  }

  public Command intakeCoral(double speed) {
    return run(
        () -> {
          // left_coral.set(speed);
          scoreMotor.set(speed);
          intakingAlgae = false;
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
