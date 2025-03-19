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

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new CoralItake. */
  // private final TalonFX left_coral = new TalonFX(4);
  private enum ScoreState {
    INTAKEALGAE("INTAKEALGAE"),
    OUTTAKEALGAE("OUTTAKEALGAE"),
    OUTTAKECORAL("OUTTAKECORAL"),
    STORECORAL("STORECORAL"),
    NONE("NONE");

    private String name;

    ScoreState(String name) {
      this.name = name;
    }

    public String getName() {
      return name;
    }
  }

  private ScoreState currentScoringState = ScoreState.NONE;

  private final TalonFX endEffectorMotor = new TalonFX(END_EFFECTOR_MOTOR_PORT);

  public EndEffectorSubsystem() {
    // in init function
    var endEffectorConfig = new TalonFXConfiguration();
    endEffectorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    endEffectorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    endEffectorConfig.CurrentLimits.StatorCurrentLimit = 40;
    endEffectorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> endEffectorMotor.getConfigurator().apply(endEffectorConfig, 0.25));

    this.setDefaultCommand(setSpeedAndState(0.0, false));
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          endEffectorMotor.set(-0.01);
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

          endEffectorMotor.set(-speed);
        });
  }

  public Command setSpeed(double speed) {
    return run(
        () -> {
          endEffectorMotor.set(speed);
        });
  }

  public boolean getCurrentSpike() {
    return endEffectorMotor.getStatorCurrent().getValueAsDouble() > 100;
  }

  public String getCurrentScoringState() {
    return currentScoringState.getName();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
