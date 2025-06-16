// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.ExtensionConstants.*;
import static frc.robot.Constants.ArmConstants.ShoulderConstants.*;
import static frc.robot.Constants.ArmConstants.WristConstants.*;
import static frc.robot.Constants.tuningMode;
import static frc.robot.util.MachineStates.HOME;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggingUtil.SimpleMotorLogger;
import frc.robot.util.MachineStates.BotState;
import org.littletonrobotics.junction.Logger;

/**
 * Arm subsystem for our shoulder, extension, and wrist baesd off of 2910's 2023/25 architecture.
 *
 * <p>Includes:
 *
 * <ul>
 *   <li>CTRE's Motion Magic tuning for all three joints.
 *   <li>Integration with the {@link frc.robot.util.MachineStates MachineStates} class for easier
 *       positing saving.
 *   <li>{@link frc.robot.util.LoggedTunableNumber LoggedTunableNumbers } for manual tuning when in
 *       {@link frc.robot.Constants#tuningMode tuningMode} (e.g. during field calibration).
 * </ul>
 *
 * See <a
 * href="https://github.com/FRCTeam2910/2023CompetitionRobot-Public/tree/main/src/main/java/org/frcteam2910/c2023/subsystems/arm">
 * 2910's arm subsystem implementation </a> for additional reference if needed
 */
public class FullArmSubsystem extends SubsystemBase {
  /** Creates a new FullArmSubsystem. */
  private LoggedTunableNumber tunableShoulder, tunableExtend, tunableWrist;

  // Motors for the SHOULDER of the arm and its accompanying MotionMagic voltage request
  private final TalonFX rightShoulderMotor = new TalonFX(SHOULDER_MOTOR_PORT_RIGHT, "static");
  private final TalonFX leftShoulderMotor = new TalonFX(SHOULDER_MOTOR_PORT_LEFT, "static");
  private final SimpleMotorLogger leftShoulderLogger =
      new SimpleMotorLogger(leftShoulderMotor, "_Arm/leftShoulder");
  private final SimpleMotorLogger rightShoulderLogger =
      new SimpleMotorLogger(rightShoulderMotor, "_Arm/rightShoulder");
  final MotionMagicVoltage shoulderRequest;

  // Motors for the EXTENSION of the arm and its accompanying MotionMagic voltage request
  private final TalonFX rightExtensionMotor = new TalonFX(EXTENSION_MOTOR_PORT_RIGHT, "static");
  private final TalonFX leftExtensionMotor = new TalonFX(EXTENSION_MOTOR_PORT_LEFT, "static");
  private final SimpleMotorLogger leftExtensionLogger =
      new SimpleMotorLogger(leftExtensionMotor, "_Arm/leftExtension");
  private final SimpleMotorLogger rightExtensionLogger =
      new SimpleMotorLogger(rightExtensionMotor, "_Arm/rightExtension");

  final MotionMagicVoltage extensionRequest;

  // Motor for the WRIST of the arm and its accompanying MotionMagic voltage request
  private final TalonFX wristMotor = new TalonFX(WRIST_MOTOR_PORT);
  private final SimpleMotorLogger wristLogger = new SimpleMotorLogger(wristMotor, "_Arm/Wrist");
  final MotionMagicVoltage wristRequest;

  private BotState currentState = HOME; // The robot's state at startup

  /**
   * A boolean that determines whether the shoulder or extension should move first. Generally, if
   * the extension is reaching a higher position, this should be true, while if the extension is
   * reaching a lower pos, false. This is to prevent tipping as much as possible.
   */
  private Boolean shoulderFirst = true;

  public FullArmSubsystem() {

    // ================
    // === Shoulder ===
    // ================
    // Software limits and configurations for the shoulder of the arm
    var shoulderConfig = new TalonFXConfiguration();
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shoulderConfig.Feedback.SensorToMechanismRatio = 210.0;
    shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.421;
    // shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    shoulderConfig.CurrentLimits.SupplyCurrentLimit = 60;
    shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shoulderConfig.CurrentLimits.StatorCurrentLimit = 60;
    shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var shoulderSlot0Configs = shoulderConfig.Slot0;

    // Set PID and FF gains and settings
    shoulderSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    shoulderSlot0Configs.kG = SHOULDER_KG;
    shoulderSlot0Configs.kV = SHOULDER_KV;
    shoulderSlot0Configs.kP = SHOULDER_KP;
    shoulderSlot0Configs.kD = SHOULDER_KD;

    // Set Motion Magic settings
    var shoulderMMConfig = shoulderConfig.MotionMagic;
    shoulderMMConfig.MotionMagicCruiseVelocity =
        SHOULDER_CRUISE_VELOCITY; // Target cruise velocity of SHOULDER_CRUISE_VELOCITY rps
    shoulderMMConfig.MotionMagicAcceleration =
        SHOULDER_ACCELERATION; // Target acceleration of SHOULDER_ACCELERATION rps/s

    // Apply the configs to both motors. The left shoulder motor should follow the right
    // since the motors are mechanically linked via a shaft
    tryUntilOk(5, () -> rightShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));
    tryUntilOk(5, () -> leftShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));
    leftShoulderMotor.setControl(new Follower(SHOULDER_MOTOR_PORT_RIGHT, true));

    // Create a Motion Magic request with a voltage output
    shoulderRequest = new MotionMagicVoltage(0);

    // =================
    // === Extension ===
    // =================

    // Software limits and configurations for the extension of the arm
    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_EXTENSION_POS;
    // extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_EXTENSION_POS;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 50;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 50;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var extensionSlot0Configs = extensionConfig.Slot0;

    // Set PID and FF gains and settings
    extensionSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    extensionSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    extensionSlot0Configs.kG = EXTENSION_KG;
    extensionSlot0Configs.kV = EXTENSION_KV;
    extensionSlot0Configs.kP = EXTENSION_KP;

    // Set Motion Magic settings
    var extensionMMConfig = extensionConfig.MotionMagic;

    extensionMMConfig.MotionMagicCruiseVelocity = EXTENSION_CRUISE_VELOCITY;
    extensionMMConfig.MotionMagicAcceleration = EXTENSION_ACCELERATION;

    // Apply the configs to both motors. The left shoulder motor should follow the right
    // since the motors are mechanically linked via a shaft
    tryUntilOk(5, () -> rightExtensionMotor.getConfigurator().apply(extensionConfig, 0.25));
    tryUntilOk(5, () -> rightExtensionMotor.getConfigurator().apply(extensionConfig, 0.25));
    leftExtensionMotor.setControl(new Follower(EXTENSION_MOTOR_PORT_RIGHT, true));

    // Create a Motion Magic request with a voltage output
    extensionRequest = new MotionMagicVoltage(0);

    // =============
    // === Wrist ===
    // =============

    // Software limits and configurations for the wrist of the arm
    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    wristConfig.Feedback.SensorToMechanismRatio = 25;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.45;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.2;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 30;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = 30;
    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Set PID and FF gains and settings
    var wristSlot0Configs = wristConfig.Slot0;
    wristSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    wristSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    wristSlot0Configs.kG = WRIST_KG;
    wristSlot0Configs.kV = WRIST_KV;
    wristSlot0Configs.kP =
        WRIST_KP; // A position error of WRIST_KP rotations results in 12 V output

    // Set Motion Magic settings
    var wristMMConfig = wristConfig.MotionMagic;
    wristMMConfig.MotionMagicCruiseVelocity =
        WRIST_CRUISE_VELOCITY; // Target cruise velocity of WRIST_CRUISE_VELOCITY rps
    wristMMConfig.MotionMagicAcceleration =
        WRIST_CRUISE_VELOCITY / 0.5; // Reach target cruise velocity in 0.5 s

    // Apply the configs to the motor
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));

    // Create a Motion Magic request with a voltage output
    wristRequest = new MotionMagicVoltage(0);

    // Create three LoggedTunableNumbers for manual tuning of each stage
    tunableShoulder = new LoggedTunableNumber("tunableShoulder", 0.05);
    tunableExtend = new LoggedTunableNumber("tunableExtend", 1.5);
    tunableWrist = new LoggedTunableNumber("tunableWrist", 0.3);
  }

  /**
   * A command the desired {@link frc.robot.util.MachineStates.BotState BotState} of the arm and
   * determines the order the arm should move.
   *
   * @param desiredState The desired BotState of the arm, which includes shoulder, extension, and
   *     wrist positions.
   * @param shoulderFirst Determines whether the shoulder should pivot first or the extension
   *     extend/retract first.
   */
  public Command setGoal(BotState desiredState, boolean shoulderFirst) {
    return run(
        () -> {
          currentState = desiredState;
          this.shoulderFirst = shoulderFirst;
        });
  }

  /**
   * A method version of the setGoal command
   *
   * @see FullArmSubsystem#setGoal setGoal Command
   */
  public void setGoalVoid(BotState desiredState, boolean shoulderFirst) {
    currentState = desiredState;

    this.shoulderFirst = shoulderFirst;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If tuning mode is on, use the manually tuned positions
    if (tuningMode && !DriverStation.isFMSAttached()) {
      setShoulder(tunableShoulder.get());
      setExtension(tunableExtend.get());
      setWrist(tunableWrist.get());
    } else {
      // If shoulderFirst is true, move the shoulder before the other subsystems
      // Otherwise, move the elevator and wrist before the shoulder

      if (shoulderFirst) {
        setShoulder(currentState.getShoulderSetpoint());

        if (shoulderSetpointReached()) {
          setExtension(currentState.getExtensionSetpoint());
          setWrist(currentState.getWristSetpoint());
        }

      } else {
        setExtension(currentState.getExtensionSetpoint());
        setWrist(currentState.getWristSetpoint());

        if (extentionSetpointReached()) {
          setShoulder(currentState.getShoulderSetpoint());
        }
      }
    }

    leftShoulderLogger.logMotorPID().logMotorPVA().logMotorSpecs().logMotorPowerData();
    rightShoulderLogger.logMotorPID().logMotorPVA().logMotorPowerData();
    leftExtensionLogger.logMotorPID().logMotorPVA().logMotorPowerData().logMotorSpecs();
    rightExtensionLogger.logMotorPID().logMotorPVA().logMotorPowerData();
    wristLogger.logMotorPID().logMotorPVA().logMotorPowerData().logMotorSpecs();

    Logger.recordOutput("_Arm/currentState", currentState.getName());
    Logger.recordOutput("_Arm/shoulderFirst", shoulderFirst);
  }

  /** Returns whether the shoulder is within 0.1 rotations (0.1 radians or ~6 degrees). */
  public boolean shoulderSetpointReached() {
    return Math.abs(
            rightShoulderMotor.getPosition().getValueAsDouble()
                - currentState.getShoulderSetpoint())
        <= 0.1;
  }

  /** Returns whether the shoulder is within 10 rotations (~1/4 of the total length of the arm). */
  public boolean extentionSetpointReached() {
    return Math.abs(
            rightExtensionMotor.getPosition().getValueAsDouble()
                - currentState.getExtensionSetpoint())
        <= 10;
  }

  /**
   * Returns the multiplier (0.5 to 1.0) of the drivetrain based on the length of the extension
   * (assuming a max position of 40).
   */
  public double getSlowDownMult() {
    return 1.0 - (rightExtensionMotor.getPosition().getValueAsDouble() * 0.0125);
  }

  /**
   * Set the shoulder to the desired position.
   *
   * @param position The position, in rotations, the shoulder should go to
   */
  public void setShoulder(double position) {
    rightShoulderMotor.setControl(shoulderRequest.withPosition(position).withSlot(0));
  }

  /**
   * Set the extension to the desired position.
   *
   * @param position The position, in rotations, the extension should go to
   */
  public void setExtension(double position) {
    rightExtensionMotor.setControl(extensionRequest.withPosition(position).withSlot(0));
  }

  /**
   * Set the wrist to the desired position.
   *
   * @param position The position, in rotations, the wrist should go to
   */
  public void setWrist(double position) {
    wristMotor.setControl(wristRequest.withPosition(position).withSlot(0));
  }

  /** Set all arm motors to Coast mode. */
  public void setCoastMode() {
    leftShoulderMotor.setNeutralMode(NeutralModeValue.Coast);
    rightShoulderMotor.setNeutralMode(NeutralModeValue.Coast);
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  /** Set all arm motors to Brake mode. */
  public void setBrakeMode() {
    leftShoulderMotor.setNeutralMode(NeutralModeValue.Brake);
    rightShoulderMotor.setNeutralMode(NeutralModeValue.Brake);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // public Command setBrakeModeCommand() {
  //   return run(
  //       () -> {
  //         var shoulderConfig = new TalonFXConfiguration();
  //         shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

  //         tryUntilOk(5, () -> rightShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));
  //         tryUntilOk(5, () -> leftShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));
  //       });
  // }
}
