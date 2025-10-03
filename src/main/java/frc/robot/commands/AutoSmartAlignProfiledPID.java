package frc.robot.commands;

import static frc.robot.util.MachineStates.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FullArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MachineStates.BotState;
import frc.robot.util.TuneableProfiledPID;

/**
 * Command that uses a {@link frc.robot.util.TuneableProfiledPID Profiled PID controller} to control
 * both the x and y speeds of the robot, along with a {@link
 * edu.wpi.first.math.controller.PIDController PID Controller} to control the heading (theta) to
 * align with the specified face, branch, and level of the reef as specified by the operator using
 * the {@link frc.robot.util.AutoScoreSelection AutoScoreSelection} class.
 */
public class AutoSmartAlignProfiledPID extends Command {

  Drive m_drive;
  AutoScoreSelection storedState;
  FullArmSubsystem arm;
  String limelightName;
  BotState currentState;

  TuneableProfiledPID profiledPid;
  PIDController thetaController;

  /** Alliance-relative (i.e. 0 for closest side to driver station). */
  double targetReefAngle = 0.0;

  double xTrans = 0.0;
  double yTrans = 0.0;
  double rotationVal = 0.0;

  /** Offsets from the AprilTag for left and right branches, in inches. */
  double targetLateralOffset = 0;

  double rightLateralOffset = 3;
  double leftLateralOffset = -0.5;

  /** Distance from the AprilTag the robot desires to go to, in inches. */
  double targetDistanceOffset = 0;

  double closeDistanceOffset = 0.33;
  double farDistanceOffset = 0.41; // L2

  /** Distance the arm can start extending from; different for L2 than L3-4, in inches. */
  double armDistanceThreshold = 0;

  double closeArmDistanceThreshold = 6;
  double farArmDistanceThreshold = 20; // L2

  boolean isFinished = false;

  BotState bState;

  public AutoSmartAlignProfiledPID(Drive d, FullArmSubsystem a, AutoScoreSelection s, BotState b) {
    this.m_drive = d;
    this.storedState = s;
    this.arm = a;
    this.bState = b;
    addRequirements(m_drive, arm);
  }

  @Override
  public void initialize() {

    isFinished = false;

    profiledPid = new TuneableProfiledPID("m_profiledPid", 0.07, 0.0, 0.0, 3, 3);
    profiledPid.setTolerance(0.4);

    thetaController = new PIDController(0.05, 0.0, 0.0);
    thetaController.enableContinuousInput(-180, 180);
    thetaController.setTolerance(3);

    targetReefAngle = storedState.getTargetReefAngle();

    if (Timer.getFPGATimestamp() - storedState.getLastUpdated() < 2.0) {
      limelightName = storedState.getTargetLimelight();
      currentState = storedState.getBotState();
    } else {
      String rightLL = Constants.LimelightConstants.rightLimelightName;
      String leftLL = Constants.LimelightConstants.leftLimelightName;
      limelightName =
          Math.abs(LimelightHelpers.getTX(rightLL)) < Math.abs(LimelightHelpers.getTX(leftLL))
              ? rightLL
              : leftLL;
      currentState = bState;
    }
    // Constants.LimelightConstants.leftLimelightName; // storedState.getTargetLimelight();

    if (limelightName == Constants.LimelightConstants.rightLimelightName) {
      targetLateralOffset = rightLateralOffset;
    } else {
      targetLateralOffset = leftLateralOffset;
    }

    // Logic for determining offsets
    targetDistanceOffset = currentState == L2 ? farDistanceOffset : closeDistanceOffset;
    armDistanceThreshold = currentState == L2 ? farArmDistanceThreshold : closeArmDistanceThreshold;
  }

  @Override
  public void execute() {

    profiledPid.updatePID();

    double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
    if (targetPose == null || targetPose.length < 3) {
      return;
    }

    double distanceToTarget = Units.metersToInches(targetPose[2] - targetDistanceOffset);
    double lateralOffset = Units.metersToInches(targetPose[0]) + targetLateralOffset;

    SmartDashboard.putNumber("smartAlign/distanceToTarget", distanceToTarget);
    SmartDashboard.putNumber("smartAlign/lateralOffset", lateralOffset);

    // Calculate the speed toward the target (i.e. if we went directly there)
    // then decompose it into x (lateral) and y (forward) translation components
    double controlValue = profiledPid.calculate(Math.hypot(distanceToTarget, lateralOffset), 1.4);
    xTrans = controlValue * (lateralOffset / Math.hypot(distanceToTarget, lateralOffset)) * 1.5;
    yTrans = controlValue * (distanceToTarget / Math.hypot(distanceToTarget, lateralOffset));

    yTrans = MathUtil.applyDeadband(yTrans, 0.1);
    xTrans = MathUtil.applyDeadband(xTrans, 0.1);

    SmartDashboard.putNumber("smartAlign/controlValue", controlValue);
    SmartDashboard.putNumber("smartAlign/hypot", Math.hypot(distanceToTarget, lateralOffset));

    rotationVal =
        thetaController.calculate(
            m_drive.getHeadingDegrees(), Math.round(m_drive.getHeadingDegrees() / 60) * 60);

    xTrans = MathUtil.clamp(xTrans, -5, 5);
    yTrans = MathUtil.clamp(yTrans, -5, 5);
    rotationVal = MathUtil.clamp(rotationVal, -0.75, 0.75);

    ChassisSpeeds speeds = new ChassisSpeeds(yTrans, -xTrans, rotationVal);
    m_drive.runVelocity(speeds);

    if (distanceToTarget < armDistanceThreshold) {
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
    return isFinished;
  }
}
