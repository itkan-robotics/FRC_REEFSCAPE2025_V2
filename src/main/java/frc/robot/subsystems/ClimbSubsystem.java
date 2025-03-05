// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CLIMB_MOTOR_PORT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final TalonFX Climb = new TalonFX(CLIMB_MOTOR_PORT);

  private Servo climblock = new Servo(0);
  private double climbSetpoint = 0.0;

  final MotionMagicVoltage m_Request;

  public ClimbSubsystem() {
    var ClimbConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var Climbslot0Configs = ClimbConfigs.Slot0;
    Climbslot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    Climbslot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    Climbslot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    Climbslot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    Climbslot0Configs.kI = 0; // no output for integrated error
    Climbslot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var ClimbmotionMagicConfigs = ClimbConfigs.MotionMagic;
    ClimbmotionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    ClimbmotionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    ClimbmotionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    Climb.getConfigurator().apply(ClimbConfigs);
    m_Request = new MotionMagicVoltage(0);
  }

  public Command setGoal(double setpoint) {
    return run(
        () -> {
          // setSetpoint(tunableHeight.get());
          setSetpoint(setpoint);
        });
  }

  public void setSetpoint(double setpoint) {
    climbSetpoint = setpoint;
    Climb.setControl(m_Request.withPosition(setpoint).withSlot(0));
  }

  public Command setServoPosition(double position) {
    return run(
        () -> {
          // setSetpoint(tunableHeight.get());
          climblock.set(position);
        });
  }

  public double getSetpoint() {
    return climbSetpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
