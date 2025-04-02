package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.util.MachineStates.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FullArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MachineStates.BotState;
import frc.robot.util.TuneableProfiledPID;

public class SmartAlignProfiledPID extends Command {

  Drive m_drive;
  AutoScoreSelection storedState;
  FullArmSubsystem arm;
  String limelightName;
  BotState currentState;

  TuneableProfiledPID m_profiledPid;
  PIDController m_thetaController;

  double targetReefAngle = 0.0;
  double xTrans = 0.0;
  double yTrans = 0.0;
  double rotationVal = 0.0;

  double rightLateralOffset = 3;
  double leftLateralOffset = -0.5;

  double closeDistanceOffset = 0.33;
  double farDistanceOffset = 0.42;

  double closeDistanceToArmOffset = 6;
  double farDistanceToArmOffset = 20;

  double distanceToArm = 0;

  double distanceOffset = 0;
  double offset = 0;

  boolean m_end = false;

  public SmartAlignProfiledPID(Drive d, FullArmSubsystem a, AutoScoreSelection b) {
    this.m_drive = d;
    this.storedState = b;
    this.arm = a;
    addRequirements(m_drive, arm);
  }

  @Override
  public void initialize() {
    currentState = storedState.getBotState();
    m_end = false;

    m_profiledPid = new TuneableProfiledPID("m_profiledPid", 0.07, 0.0, 0.0, 3, 3);
    m_profiledPid.setTolerance(0.4);

    m_thetaController = new PIDController(0.05, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setTolerance(3);

    targetReefAngle = storedState.getTargetReefAngle();

    limelightName = storedState.getTargetLimelight();
    // Constants.LimelightConstants.leftLimelightName; // storedState.getTargetLimelight();

    if (limelightName == Constants.LimelightConstants.rightLimelightName) {
      offset = rightLateralOffset;
    } else {
      offset = leftLateralOffset;
    }

    distanceOffset = currentState == L2 ? farDistanceOffset : closeDistanceOffset;
    distanceToArm = currentState == L2 ? farDistanceToArmOffset : closeDistanceToArmOffset;
  }

  @Override
  public void execute() {

    m_profiledPid.updatePID();

    double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
    if (targetPose == null || targetPose.length < 3) {
      return;
    }

    double distanceToTarget = Units.metersToInches(targetPose[2] - distanceOffset);
    double lateralOffset = Units.metersToInches(targetPose[0]) + offset;

    SmartDashboard.putNumber("smartAlign/distanceToTarget", distanceToTarget);
    SmartDashboard.putNumber("smartAlign/lateralOffset", lateralOffset);

    double controlValue = m_profiledPid.calculate(Math.hypot(distanceToTarget, lateralOffset), 1.4);
    xTrans = controlValue * (lateralOffset / Math.hypot(distanceToTarget, lateralOffset)) * 1.5;
    yTrans = controlValue * (distanceToTarget / Math.hypot(distanceToTarget, lateralOffset));

    yTrans = MathUtil.applyDeadband(yTrans, 0.1);
    xTrans = MathUtil.applyDeadband(xTrans, 0.1);
    rotationVal = MathUtil.applyDeadband(rotationVal, 0.05);

    SmartDashboard.putNumber("smartAlign/controlValue", controlValue);
    SmartDashboard.putNumber("smartAlign/hypot", Math.hypot(distanceToTarget, lateralOffset));

    rotationVal =
        m_thetaController.calculate(
            m_drive.getHeadingDegrees(), Math.round(m_drive.getHeadingDegrees() / 60) * 60);

    xTrans = MathUtil.clamp(xTrans, -5, 5);
    yTrans = MathUtil.clamp(yTrans, -5, 5);
    rotationVal = MathUtil.clamp(rotationVal, -0.75, 0.75);

    ChassisSpeeds speeds = new ChassisSpeeds(yTrans, -xTrans, rotationVal);
    m_drive.runVelocity(speeds);

    if (distanceToTarget < distanceToArm) {
      arm.setGoalVoid(currentState, true);
    }

    SmartDashboard.putNumber("smartAlign/CombinedProfiledPID X", xTrans);
    SmartDashboard.putNumber("smartAlign/CombinedProfiledPID Y", yTrans);
    SmartDashboard.putNumber("smartAlign/RotationValue", rotationVal);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return m_end;
  }
}
