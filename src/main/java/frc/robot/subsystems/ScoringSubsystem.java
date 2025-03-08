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
  private enum ScoreState {
    INTAKEALGAE,
    OUTTAKEALGAE,
    OUTTAKECORAL,
    STORECORAL,
    NONE
  }

  private ScoreState currentScoringState = ScoreState.NONE;

  private final TalonFX scoreMotor = new TalonFX(SCORE_MOTOR_PORT);

  public ScoringSubsystem() {
    // in init function
    var scoringConfig = new TalonFXConfiguration();
    scoringConfig.CurrentLimits.SupplyCurrentLimit = 40;
    scoringConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    scoringConfig.CurrentLimits.StatorCurrentLimit = 40;
    scoringConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> scoreMotor.getConfigurator().apply(scoringConfig, 0.25));

    this.setDefaultCommand(setSpeedAndState(0.005, false));
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          scoreMotor.set(-0.01);
        });
  }

  /**
   * Sets the speed of the outtake. A negative value means the coral outtakes/algae intakes, while a
   * positive value means the coral doesn't outtake / algae outtakes
   *
   * @param speed values explained in description
   * @param isAlgae boolean that is used to determine what state the scoring subsystem is in (i.e.
   *     intaking/outtaking coral/algae)
   */
  public Command setSpeedAndState(double speed, boolean isAlgae) {
    return run(
        () -> {
          if (isAlgae && speed <= 0.0) {
            currentScoringState = ScoreState.INTAKEALGAE;

          } else if (isAlgae && speed > 0.0) {
            currentScoringState = ScoreState.OUTTAKEALGAE;

          } else if (speed >= 0.0) {
            currentScoringState = ScoreState.STORECORAL;

          } else {
            currentScoringState = ScoreState.OUTTAKECORAL;
          }

          scoreMotor.set(getCurrentSpike() ? 0.01 : -speed);
        });
  }

  // public Command intakeAlgae(double speed) {
  //   return run(
  //       () -> {
  //         scoreMotor.set(getCurrentSpike() ? 0.05 : speed);
  //         intakingAlgae = true;
  //       });
  // }

  // public Command intakeCoral(double speed) {
  //   return run(
  //       () -> {
  //         // left_coral.set(speed);
  //         scoreMotor.set(speed);
  //         intakingAlgae = false;
  //       });
  // }

  public boolean getCurrentSpike() {
    return scoreMotor.getStatorCurrent().getValueAsDouble() > 100;
  }

  public String getCurrentScoringState() {
    switch (currentScoringState) {
      case NONE:
        return "NONE";
      case INTAKEALGAE:
        return "INTAKING ALGAE";
      case OUTTAKEALGAE:
        return "OUTTAKING ALGAE";
      case OUTTAKECORAL:
        return "OUTTAKING ALGAE";
      case STORECORAL:
        return "STORING CORAL";
      default:
        return "HOW DID WE GET HERE";
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
