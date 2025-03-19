// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.util.MachineStates.PossibleStates;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import frc.robot.subsystems.arm.ShoulderSubsystem;
import frc.robot.subsystems.arm.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MachineStates.BotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartAlign extends Command {
  /** Creates a new SmartAlign. */
  Drive drive;

  ShoulderSubsystem shoulder;
  ExtensionSubsystem extension;
  WristSubsystem wrist;

  String limelightName;
  AutoScoreSelection storedState;
  BotState currentState;

  PIDController m_pidControllerY, m_pidControllerX, m_thetaController;

  boolean isAngleReached = true;
  boolean isStrafeReached = false;

  double targetReefAngle = 0.0;

  double omega = 0.0;
  double xTrans = 0.0;
  double yTrans = 0.0;
  double count = 0.0;
  double rotationVal = 0.0;

  boolean m_end = false;
  int tagID = 10;
  double reefAngle = 0.0;

  boolean isFlipped;

  public SmartAlign(
      Drive drive,
      ShoulderSubsystem shoulder,
      ExtensionSubsystem e,
      WristSubsystem wrist,
      EndEffectorSubsystem s,
      AutoScoreSelection storedState) {
    this.drive = drive;
    this.extension = e;
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.storedState = storedState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, extension, shoulder, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // System.out.println("0");
    currentState = PossibleStates[storedState.getBotStateInt()];

    m_end = false;
    limelightName = storedState.getTargetLimelight();
    
    m_pidControllerY = new PIDController(0.075, 0, 0);
    m_pidControllerY.setTolerance(0.5);

    m_pidControllerX = new PIDController(0.045, 0, 0);
    m_pidControllerX.setTolerance(0.5);

    m_thetaController = new PIDController(0.12, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setTolerance(3);

    targetReefAngle =
        MathUtil.inputModulus(storedState.getTargetReefAngle() + (isFlipped ? 180 : 0), -180, 180);
    // targetReefAngle = storedState.getTargetReefAngle();
    // targetReefAngle = 180.0;
    // System.out.println("1");
    reefAngle = LimelightSubsystem.getLLReefAngle(limelightName);
    // System.out.println("2");

    // isAngleReached = true;
    isStrafeReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // System.out.println("3");
    // if (!isAngleReached) {
    //   // System.out.println("5");
    //   // System.out.println(m_drive.getRotation().getDegrees());
    //   if ((Math.abs(targetReefAngle) == 180.0
    //           ? Math.abs(Math.abs(m_drive.getRotation().getDegrees()) - 180)
    //           : Math.abs(m_drive.getRotation().getDegrees() - targetReefAngle))
    //       < 2.5) {
    //     // System.out.println("5.1");
    //     isAngleReached = true;
    //   }
    //   rotationVal =
    //       m_thetaController.calculate(m_drive.getRotation().getDegrees(), targetReefAngle);
    //   // System.out.println("5.2");
    //   m_thetaController.calculate(
    //       (MathUtil.inputModulus(m_drive.getHeadingDegrees(), -180, 180)), targetReefAngle);
    //   // System.out.println("5.4");
    //   // rotationVal = MathUtil.clamp(rotationVal, -0.3, 0.3);
    //   // //System.out.println("6");
    //   // If it is reached, start moving towards the target
    //   ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, rotationVal);
    //   m_drive.runVelocity(speeds);
    // }
    if (LimelightHelpers.getTV(limelightName)) {
      // System.out.println("4"); // If the angle isn't reached, keep rotating until it does
      if (isAngleReached) {
        // //System.out.println("7");
        // rotationVal =
        //     m_thetaController.calculate(m_drive.getRotation().getDegrees(), targetReefAngle);
        m_pidControllerX.setSetpoint(0);
        // //System.out.println("8");
        xTrans = m_pidControllerX.calculate(LimelightHelpers.getTX(limelightName));
        xTrans = MathUtil.clamp(xTrans, -5, 5);

        m_pidControllerY.setSetpoint(18.5);
        // //System.out.println("9");
        yTrans = m_pidControllerY.calculate(LimelightHelpers.getTA(limelightName));
        // //System.out.println("10");
        yTrans = MathUtil.clamp(yTrans, -5, 5);

        // If x-values within tolerance, start superstructuring
        if (isStrafeReached) {
          // //System.out.println("11");
          extension.setSetpoint(currentState.getExtensionSetpoint());
          shoulder.setSetpoint(currentState.getShoulderSetpoint());
          // If not reached, check if it is reached
        } else {
          // //System.out.println("12");
          if (Math.abs(m_pidControllerX.calculate(LimelightHelpers.getTX(limelightName))) < 0.25) {
            isStrafeReached = true;
          }
        }
        // System.out.println(LimelightHelpers.getTX(limelightName));
      }

      // Do all the speeds stuff

      if (isAngleReached) {
        MathUtil.clamp(rotationVal, -0.75, 0.75);
      }
      if (LimelightHelpers.getTA(limelightName) > 10.0) {
        MathUtil.clamp(yTrans, -0.3, 0.3);
      }

      ChassisSpeeds speeds = new ChassisSpeeds(yTrans, xTrans, 0.0);

      drive.runVelocity(speeds);

      if (m_pidControllerX.atSetpoint() && m_pidControllerY.atSetpoint() && count > 25) {
        m_end = true;
      } else {
        m_end = false;
      }
    }

    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
