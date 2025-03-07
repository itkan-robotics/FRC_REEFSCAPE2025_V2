// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class FingerSubsystem extends SubsystemBase {
  /** Creates a new FingerSubsystem. */
  // private Servo climbServo = new Servo(FingerConstants.FINGER_SERVO_PORT);

  private LoggedTunableNumber tunableAngle;

  public FingerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // climbServo.set(tunableAngle.get());
  }

  public Command setFingerOut() {
    return run(
        () -> {
          // climbServo.set(FingerConstants.FINGER_OUT_POS);
        });
  }

  public Command setFingerIn() {
    return run(
        () -> {
          // climbServo.set(FingerConstants.FINGER_IN_POS);
        });
  }
}
