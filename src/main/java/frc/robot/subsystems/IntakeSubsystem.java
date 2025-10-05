// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Intake_Motor_Port;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
// import com.playingwithfusion.TimeOfFlight;
// import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LoggingUtil.SimpleMotorLogger;
import org.littletonrobotics.junction.Logger;

/** Subsystem for the end effector */
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeMotor = new TalonFX(Intake_Motor_Port);

  private double desiredIntakeSpeed = 0.045;

  private Debouncer gamepieceDebouncerRising;
  private Debouncer gamepieceDebouncerFalling;
  private boolean gamepieceDetected = false;

  private final SimpleMotorLogger intakeLogger =
      new SimpleMotorLogger(intakeMotor, "_Intake/motor");

  public enum IntakeState {
    NO_GAMEPIECE,
    HAS_CORAL,
    HAS_ALGAE,
    INTAKING_CORAL,
    INTAKING_ALGAE,
    OUTTAKING_CORAL,
    OUTTAKING_ALGAE,
    DEALGAEFYING
  }

  IntakeState currentIntakeState = IntakeState.NO_GAMEPIECE;

  // private TimeOfFlight intake_sensor = new TimeOfFlight(0);
  DigitalInput ranger = new DigitalInput(0);

  public IntakeSubsystem() {

    // Configure the intake motor
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

    // Configure the intake sensor
    // intake_sensor.setRangingMode(RangingMode.Short, 40);
    gamepieceDebouncerRising = new Debouncer(0.05, DebounceType.kRising);
    gamepieceDebouncerFalling = new Debouncer(0.5, DebounceType.kFalling);
  }

  public void tryState(IntakeState desiredState) {
    switch (desiredState) {
      case INTAKING_CORAL:
        currentIntakeState = IntakeState.INTAKING_CORAL;
        break;
      case INTAKING_ALGAE:
        currentIntakeState = IntakeState.INTAKING_ALGAE;
        break;
      case OUTTAKING_CORAL:
        currentIntakeState = IntakeState.OUTTAKING_CORAL;
        break;
      case OUTTAKING_ALGAE:
        currentIntakeState = IntakeState.OUTTAKING_ALGAE;
        break;
      case DEALGAEFYING:
        currentIntakeState = IntakeState.DEALGAEFYING;
      default:
        break;
    }
  }

  public void applyState() {
    switch (currentIntakeState) {
      case INTAKING_CORAL:
      case INTAKING_ALGAE:
        desiredIntakeSpeed = 1.0;
        break;
      case OUTTAKING_CORAL:
      case OUTTAKING_ALGAE:
      case DEALGAEFYING:
        desiredIntakeSpeed = -1.0;
        break;
      case HAS_ALGAE:
        desiredIntakeSpeed = 0.5;
        break;
      case HAS_CORAL:
        desiredIntakeSpeed = 0.045;
        break;
      default:
        break;
    }
  }

  /** By default, run the intake at 4.5% speed */
  public Command DefaultCommand() {
    return run(
        () -> {
          setIntakeDutyCycle(desiredIntakeSpeed);
        });
  }

  /**
   * Simple setter Command for the intake
   *
   * @param speed The desired speed
   */
  public Command setIntakeSpeed(double speed) {
    return run(
        () -> {
          setIntakeDutyCycle(speed);
        });
  }

  /**
   * Set the outtake speed based on the current stored state set by the operator.
   *
   * @since 4/19/2025: L2 at 20% speed, L1 at 50% speed, everything else at 80% speed
   * @param storedState The stored state set by the operator.
   */
  public Command outtakeBotState(AutoScoreSelection storedState) {
    return run(
        () -> {
          var currentState = storedState.getBotState();
          intakeMotor.set(currentState.getOuttakeSpeed());
        });
  }

  /**
   * Simple setter method for the intake
   *
   * @param speed
   */
  public void setIntakeDutyCycle(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Method for checking if coral is intaked fully.
   *
   * @deprecated
   */
  @Deprecated
  public boolean isIntaked() {
    // return intake_sensor.getRange() < 100;
    return !ranger.get();
  }

  public boolean gamepieceDetected() {
    return Math.abs(intakeMotor.getVelocity().getValueAsDouble()) <= 0.02
        && intakeMotor.getSupplyCurrent().getValueAsDouble() >= 1;
  }

  public boolean getGpDetected() {
    return gamepieceDetected;
  }

  public Command intakeUntilStalled() {
    return run(() -> {
          setIntakeSpeed(1.0);
        })
        .until(() -> gamepieceDetected())
        .andThen(
            new InstantCommand(
                () -> {
                  setIntakeSpeed(0.4);
                }));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("ranger/Ranger Value", ranger.get());
    gamepieceDetected =
        gamepieceDebouncerRising.calculate(gamepieceDetected())
            || gamepieceDebouncerFalling.calculate(gamepieceDetected());
    intakeLogger.logMotorSpecs().logMotorPowerData();
    Logger.recordOutput("_Intake/ranger/Ranger Value", ranger.get());

    Logger.recordOutput("_Intake/gpDetectedUnfiltered", gamepieceDetected());
    Logger.recordOutput("_Intake/gpDetectedFiltered", gamepieceDetected);

    if (gamepieceDetected) {
      currentIntakeState = IntakeState.HAS_CORAL;
    } else {
      currentIntakeState = IntakeState.NO_GAMEPIECE;
    }
  }

  public boolean isIntakeAtDesiredState() {
    return true;
  }
}
