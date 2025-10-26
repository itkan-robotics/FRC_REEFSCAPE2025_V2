// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ALGAE_MOTOR_PORT;
import static frc.robot.Constants.CORAL_LEFT_MOTOR_PORT;
import static frc.robot.Constants.CORAL_RIGHT_MOTOR_PORT;
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
  private final TalonFX coralMotorLeft = new TalonFX(CORAL_LEFT_MOTOR_PORT);

  private final TalonFX coralMotorRight = new TalonFX(CORAL_RIGHT_MOTOR_PORT);
  private final TalonFX algaeMotor = new TalonFX(ALGAE_MOTOR_PORT);

  private double desiredIntakeSpeed = 0.045;

  private Debouncer coralDetectionDebouncerRising;
  private Debouncer coralDetectionDebouncerFalling;
  private boolean coralDetectedWithDebouncer = false;

  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  private final SimpleMotorLogger intakeLogger =
      new SimpleMotorLogger(coralMotorLeft, "_Intake/motor");

  public enum IntakeState {
    NO_GAMEPIECE,
    HAS_CORAL,
    HAS_ALGAE,
    INTAKING_CORAL,
    INTAKING_CORAL_L1,
    INTAKING_ALGAE,
    OUTTAKING_CORAL,
    OUTTAKING_ALGAE,
    DEALGAEFYINGLOW,
    DEALGAEFYINGHIGH,
    IDLE
  }

  IntakeState currentIntakeState = IntakeState.NO_GAMEPIECE;

  // private TimeOfFlight intake_sensor = new TimeOfFlight(0);
  DigitalInput ranger = new DigitalInput(0);

  public IntakeSubsystem() {

    // Configure the intake motor
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 30;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> coralMotorLeft.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> coralMotorRight.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> algaeMotor.getConfigurator().apply(intakeConfig, 0.25));

    // Configure the intake sensor
    // intake_sensor.setRangingMode(RangingMode.Short, 40);
    coralDetectionDebouncerRising = new Debouncer(0.05, DebounceType.kRising);
    coralDetectionDebouncerFalling = new Debouncer(0.5, DebounceType.kFalling);
  }

  public void tryState(IntakeState desiredState) {
    // System.out.println("Intake state: " + desiredState);
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
      case DEALGAEFYINGLOW:
        currentIntakeState = IntakeState.DEALGAEFYINGLOW;
        break;
      case DEALGAEFYINGHIGH:
        currentIntakeState = IntakeState.DEALGAEFYINGHIGH;
        break;
      case IDLE:
        currentIntakeState = IntakeState.IDLE;
        break;
      default:
        break;
    }
  }

  public void applyState() {
    switch (currentIntakeState) {
      case INTAKING_CORAL:
      case OUTTAKING_ALGAE:
      case DEALGAEFYINGHIGH:
        desiredIntakeSpeed = 0.7;
        break;
      case OUTTAKING_CORAL:
      case INTAKING_ALGAE:
      case DEALGAEFYINGLOW:
        desiredIntakeSpeed = -1.0;
        break;
      case HAS_ALGAE:
        desiredIntakeSpeed = 0.5;
        break;
      case HAS_CORAL:
        desiredIntakeSpeed = 0.045;
        break;
      case IDLE:
        desiredIntakeSpeed = 0.2;
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

  public void setToggleStatorCurrentLimit(boolean toggle) {
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = toggle;
    coralMotorLeft.getConfigurator().apply(intakeConfig, 0.25);
    coralMotorRight.getConfigurator().apply(intakeConfig, 0.25);
    algaeMotor.getConfigurator().apply(intakeConfig, 0.25);
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
          coralMotorLeft.set(currentState.getOuttakeSpeed());
        });
  }

  /**
   * Simple setter method for the intake
   *
   * @param speed
   */
  public void setIntakeDutyCycle(double speed) {
    coralMotorLeft.set(-speed);
    coralMotorRight.set(speed);
    algaeMotor.set(-speed);
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

  public boolean coralDetectedInstant() {
    return currentIntakeState == IntakeState.INTAKING_CORAL
        ? Math.abs(coralMotorLeft.getVelocity().getValueAsDouble()) <= 80.0
                && coralMotorLeft.getStatorCurrent().getValueAsDouble() >= 15.0
            || Math.abs(coralMotorRight.getVelocity().getValueAsDouble()) <= 80.0
                && coralMotorRight.getStatorCurrent().getValueAsDouble() >= 15.0
        : false;
  }

  public boolean getCoralDetectedWithDebouncer() {
    return coralDetectedWithDebouncer;
  }

  public Command intakeUntilStalled() {
    return run(() -> {
          setIntakeSpeed(1.0);
        })
        .until(() -> coralDetectedInstant())
        .andThen(
            new InstantCommand(
                () -> {
                  setIntakeSpeed(0.4);
                }));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("ranger/Ranger Value", ranger.get());
    coralDetectedWithDebouncer =
        coralDetectionDebouncerRising.calculate(coralDetectedInstant())
            || coralDetectionDebouncerFalling.calculate(coralDetectedInstant());
    intakeLogger.logMotorSpecs().logMotorPowerData();
    Logger.recordOutput("_Intake/ranger/Ranger Value", ranger.get());

    Logger.recordOutput("_Intake/gpDetectedUnfiltered", coralDetectedInstant());
    Logger.recordOutput("_Intake/gpDetectedFiltered", coralDetectedWithDebouncer);
    Logger.recordOutput("_Intake/desiredSpeed", desiredIntakeSpeed);

    // if (coralDetectedInstant()) {
    //   currentIntakeState = IntakeState.HAS_CORAL;
    // } else {
    setIntakeDutyCycle(desiredIntakeSpeed);
    // }

    applyState();
  }

  public boolean isIntakeAtDesiredState() {
    return true;
  }
}
