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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MachineStates.BotState;

public class FullArmSubsystem extends SubsystemBase {
  /** Creates a new FullArmSubsystem. */
  private LoggedTunableNumber tunableShoulder, tunableExtend, tunableWrist;

  // ARM
  private final TalonFX rightShoulderMotor = new TalonFX(SHOULDER_MOTOR_PORT, "static");

  private final TalonFX leftShoulderMotor = new TalonFX(LEFT_SHOULDER_MOTOR_PORT, "static");
  final MotionMagicVoltage arm_mRequest;

  // EXTEND
  private final TalonFX extensionMotorRight = new TalonFX(EXTENSION_MOTOR_PORT_RIGHT, "static");
  private final TalonFX extensionMotorLeft = new TalonFX(EXTENSION_MOTOR_PORT_LEFT, "static");
  final MotionMagicVoltage elevator_mRequest;

  // WRIST
  private final TalonFX wristMotor = new TalonFX(WRIST_MOTOR_PORT_A);
  final MotionMagicVoltage m_lRequest;

  private BotState currentState = HOME;
  private Boolean shoulderFirst = true;

  public FullArmSubsystem() {
    // in init function
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

    // set slot 0 gains
    shoulderSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    // shoulderSlot0Configs.kS = SHOULDER_KS; // Add SHOULDER_KS V output to overcome static
    // friction
    shoulderSlot0Configs.kG = SHOULDER_KG;
    shoulderSlot0Configs.kV = SHOULDER_KV;
    shoulderSlot0Configs.kP =
        SHOULDER_KP; // A position error of SHOULDER_KP rotations results in 12 V output
    shoulderSlot0Configs.kD = SHOULDER_KD;

    // set Motion Magic settings
    var shoulderMMConfig = shoulderConfig.MotionMagic;
    shoulderMMConfig.MotionMagicCruiseVelocity =
        SHOULDER_CRUISE_VELOCITY; // Target cruise velocity of SHOULDER_CRUISE_VELOCITY rps
    shoulderMMConfig.MotionMagicAcceleration =
        SHOULDER_ACCELERATION; // Target acceleration of SHOULDER_ACCELERATION rps/s

    // in init function
    tryUntilOk(5, () -> rightShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));
    tryUntilOk(5, () -> leftShoulderMotor.getConfigurator().apply(shoulderConfig, 0.25));

    // tryUntilOk(5, () -> rightShoulderMotor.setPosition(SHOULDER_ZERO_POSITION, 0.25));
    // tryUntilOk(5, () -> leftShoulderMotor.setPosition(SHOULDER_ZERO_POSITION, 0.25));

    leftShoulderMotor.setControl(new Follower(SHOULDER_MOTOR_PORT, true));

    // create a Motion Magic request, voltage output
    arm_mRequest = new MotionMagicVoltage(0);

    // ELEVATOR

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

    // set slot 0 gains
    extensionSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    extensionSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    extensionSlot0Configs.kG = EXTENSION_KG;
    extensionSlot0Configs.kV = EXTENSION_KV;
    extensionSlot0Configs.kP =
        EXTENSION_KP; // A position error of EXTENSION_KP rotations results in 12 V output

    // set Motion Magic settings
    var extensionMMConfig = extensionConfig.MotionMagic;

    extensionMMConfig.MotionMagicCruiseVelocity = EXTENSION_CRUISE_VELOCITY;
    extensionMMConfig.MotionMagicAcceleration = EXTENSION_ACCELERATION;

    // in init function
    tryUntilOk(5, () -> extensionMotorRight.getConfigurator().apply(extensionConfig, 0.25));
    tryUntilOk(5, () -> extensionMotorLeft.getConfigurator().apply(extensionConfig, 0.25));

    // tryUntilOk(5, () -> extensionMotorRight.setPosition(EXTENSION_ZERO_POSITION, 0.25));
    // tryUntilOk(5, () -> extensionMotorLeft.setPosition(EXTENSION_ZERO_POSITION, 0.25));

    extensionMotorLeft.setControl(new Follower(EXTENSION_MOTOR_PORT_RIGHT, true));
    elevator_mRequest = new MotionMagicVoltage(0);

    // WIRST

    // in init function
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

    var wristSlot0Configs = wristConfig.Slot0;

    // set slot 0 gains
    wristSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    wristSlot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    wristSlot0Configs.kG = WRIST_KG;
    wristSlot0Configs.kV = WRIST_KV;
    wristSlot0Configs.kP =
        WRIST_KP; // A position error of WRIST_KP rotations results in 12 V output

    // set Motion Magic settings
    var wristMMConfig = wristConfig.MotionMagic;
    wristMMConfig.MotionMagicCruiseVelocity =
        WRIST_CRUISE_VELOCITY; // Target cruise velocity of WRIST_CRUISE_VELOCITY rps
    wristMMConfig.MotionMagicAcceleration =
        WRIST_CRUISE_VELOCITY / 0.5; // Reach target cruise velocity in 0.5 s

    // in init function
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));
    // tryUntilOk(5, () -> wristMotor.setPosition(WRIST_ZERO_POSITION, 0.25));

    // create a Motion Magic request, voltage output
    m_lRequest = new MotionMagicVoltage(0);

    tunableShoulder = new LoggedTunableNumber("tunableShoulder", 0.0);
    tunableExtend = new LoggedTunableNumber("tunableExtend", 0.0);
    tunableWrist = new LoggedTunableNumber("tunableWrist", 0.0);
  }

  public Command setGoal(BotState desiredState, boolean shoulderFirst) {
    return run(
        () -> {
          currentState = desiredState;
          this.shoulderFirst = shoulderFirst;
        });
  }

  public void setGoalVoid(BotState desiredState, boolean shoulderFirst) {
    currentState = desiredState;

    this.shoulderFirst = shoulderFirst;
  }

  @Override
  public void periodic() {

    if (tuningMode) {
      setShoulder(tunableShoulder.get());
      setExtention(tunableExtend.get());
      setWrist(tunableWrist.get());
    } else {
      if (shoulderFirst) {
        setShoulder(currentState.getShoulderSetpoint());

        if (ShoulderSetpointReached()) {
          setExtention(currentState.getExtensionSetpoint());
          setWrist(currentState.getWristSetpoint());
        }

      } else {
        setExtention(currentState.getExtensionSetpoint());
        setWrist(currentState.getWristSetpoint());

        if (ExtentionSetpointReached()) {
          setShoulder(currentState.getShoulderSetpoint());
        }
      }
    }
    // This method will be called once per scheduler run
  }

  public Command setWrist(BotState desiredState) {
    return run(
        () -> {
          wristMotor.setControl(
              m_lRequest.withPosition(desiredState.getWristSetpoint()).withSlot(0));
        });
  }

  public boolean ShoulderSetpointReached() {
    return Math.abs(
            rightShoulderMotor.getPosition().getValueAsDouble()
                - currentState.getShoulderSetpoint())
        <= 0.1;
  }

  public boolean ExtentionSetpointReached() {
    return Math.abs(
            extensionMotorRight.getPosition().getValueAsDouble()
                - currentState.getExtensionSetpoint())
        <= 10;
  }

  public double getSlowDownMult() {
    return 1.0 - (extensionMotorRight.getPosition().getValueAsDouble() * 0.0125);
  }

  public void setShoulder(double position) {
    rightShoulderMotor.setControl(arm_mRequest.withPosition(position).withSlot(0));
  }

  public void setExtention(double position) {
    extensionMotorRight.setControl(elevator_mRequest.withPosition(position).withSlot(0));
  }

  public void setWrist(double position) {
    wristMotor.setControl(m_lRequest.withPosition(position).withSlot(0));
  }

  // public void setCoastMode() {
  //   leftShoulderMotor.setNeutralMode(NeutralModeValue.Coast);
  //   rightShoulderMotor.setNeutralMode(NeutralModeValue.Coast);
  //   wristMotor.setNeutralMode(NeutralModeValue.Coast);
  // }

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
