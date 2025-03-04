// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.BotState;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TuneableFFPID;

/*************************************************************************************
 * Field centric drive command using targetLimelight target data to
 * calculate the desired angle of the robot to align parallel to the target AprilTag
 * and move to the AprilTag based on the target's ta and tx values using Profiled PID.
 * <p> Last Updated by Abdullah Khaled, 3/3/2025
 *************************************************************************************/
public class DriveToReefCommand extends Command {
  Drive drive;
  ElevatorSubsystem elevator;
  AutoScoreSelection storedState;
  String llName;
  
  double slowDownMult = 1.0;
  double reefAngle = 0.0;
  
  PIDController angleController, horizontalController, rangeController;
  TuneableFFPID horizontalFFController, rangeFFController;
  LoggedTunableNumber horizontal_kP, range_kP, horizontal_kS, range_kS;
  
  double omega = 0.0;
  Translation2d simpleLinearVelocity = new Translation2d();

  boolean finished = false;

  /** Creates a new DriveToReefCommand. */
  public DriveToReefCommand(Drive d, AutoScoreSelection storedState, ElevatorSubsystem e) {
    this.drive = d;
    this.storedState = storedState;
    elevator = e;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Log the slow down multiplier found from the elevator's height
    BotState storedBotState = Constants.toBotState(storedState.getBotStateInt());
    slowDownMult = elevator.getSlowDownMult(storedBotState.getElevatorSetpoint());
    SmartDashboard.putNumber("slowdownmult", slowDownMult);

    // Create PID controller for angle, horizontal, and range
    angleController = new PIDController(TURN_KP, 0.0, 0.0);
    angleController.enableContinuousInput(-180, 180);

    // // horizontal_kP = new LoggedTunableNumber("simpleReefAlignment/Horizontal kP", 0.0);
    // // horizontal_kS = new LoggedTunableNumber("simpleReefAlignment/Horizontal kS", 0.0);

    // // range_kP = new LoggedTunableNumber("simpleReefAlignment/Range kP", 0.0);
    // // range_kS = new LoggedTunableNumber("simpleReefAlignment/Range kS", 0.0);

    // horizontalController = new PIDController(horizontal_kP.getAsDouble(), 0.0, 0.0);
    // rangeController = new PIDController(range_kP.getAsDouble(), 0.0, 0.0);

    // Trying my new Tuneable PID Controller with a feedforward term!
    horizontalFFController =
        new TuneableFFPID("simpleReefAlignment/horizontalController", 0.0, 0.0, 0.0, 0.0);
    rangeFFController = new TuneableFFPID("simpleReefAlignment/rangeController", 0.0, 0.0, 0.0, 0.0);

    /* Get the target pipeline as an integer
     * If equal to the left branch, use the right limelight (b/c that's the one that
     * will be in view of the AprilTag). Vice-versa for the right branch 
    */
    double targetLimelightInt = storedState.getLimelightTargetPipeline();

    if (targetLimelightInt == LEFT_BRANCH_PIPELINE) {
      llName = LimelightConstants.rightLimelightName;
      // SmartDashboard.putNumber("limelightAlign/int", LEFT_BRANCH_PIPELINE);
    } else if (targetLimelightInt == RIGHT_BRANCH_PIPELINE) {
      llName = LimelightConstants.leftLimelightName;
      // SmartDashboard.putNumber("limelightAlign/int", RIGHT_BRANCH_PIPELINE);
    }

    // Log the target angle for the robot based on the AprilTag ID
    reefAngle = LimelightSubsystem.getLLReefAngle(llName);
    SmartDashboard.putNumber("simpleReefAlignment/Reef Angle", reefAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update the PID values of the controllers (experimental)
    horizontalFFController.updatePID();
    rangeFFController.updatePID();

    // Log the target angle for the robot based on the AprilTag ID
    reefAngle = LimelightSubsystem.getLLReefAngle(llName);
    SmartDashboard.putNumber("simpleReefAlignment/Reef Angle", reefAngle);

    /** If 
     * a. The limelight sees a target 
     * b. The primary target ID of the limelight is the same as the one on either blue/red alliance
     * c. The angle of the robot is within +-5 degrees of the reef's angle
     * continue onwards! */
    if (LimelightHelpers.getTV(llName)
        && (LimelightHelpers.getFiducialID(llName) == storedState.getTargetAprilTag()
            || LimelightHelpers.getFiducialID(llName) == storedState.getTargetAprilTag() + 11)
        && Math.abs(drive.getRotation().getDegrees() - reefAngle) < 10.0) {

      // SmartDashboard.putString("limelightAlign/target", "can see");

      // Calculate angular speed
      omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);

      // simpleLinearVelocity =
      //     new Translation2d(
      //         horizontalController.calculate(LimelightHelpers.getTX(llName), 0.0) +
      // horizontal_kS.get(),
      //         rangeController.calculate(LimelightHelpers.getTA(llName), MAX_AREA) +
      // range_kS.get());

      /**
       * Calculate the velocity of the robot 
       * First, the horizontal movement is calculated using the TX (horizonal angle) value
       * of the camera (since we assume the robot is aligned ~95%)
       * Second, the vertical movement is calculated using the TA (Area) value
       * of the camera (bigger area, closer to target); this term has a setpoint of the 
       * area we see when we are right up against the reef (~16%)
       * Finally, we rotate this velocity by the angle of the reef we are driving to
       * since these calculations are always relative to the robot/AprilTag
       * - Abdullah Khaled's explanation (hope it makes sense!)
      */
      simpleLinearVelocity =
          new Translation2d(
              horizontalFFController.calculateWithFF(LimelightHelpers.getTX(llName), 0.0),
              rangeFFController.calculateWithFF(LimelightHelpers.getTA(llName), MAX_AREA));

      simpleLinearVelocity.rotateBy(
          new Rotation2d(Math.toRadians(storedState.getTargetReefAngle())));

      /**
       * Log the values of TX and TA (should be viewed as a graph in Elastic)
       * This should be used to tune the horizontal and range PIDF values
      */
      //TO-DO: Tune values
      SmartDashboard.putNumber("simpleReefAlignment/TX", LimelightHelpers.getTX(llName));
      SmartDashboard.putNumber("simpleReefAlignment/TA", LimelightHelpers.getTA(llName));

      // Correct angle once location reached or can't see AprilTag yet or angle too sharp
    } else if (reefAngle == storedState.getTargetReefAngle()) {
      omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);
    }

    /**Calculate the scalar of the drive based on 
     * a. The actual max speed of the robot
     * b. The slowdown multiplier based on the target elevator height
    */
    double driveScalar = drive.getMaxLinearSpeedMetersPerSec() * slowDownMult;

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            simpleLinearVelocity.getX() * driveScalar,
            simpleLinearVelocity.getY() * driveScalar,
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    //Log the speeds of the robot
    SmartDashboard.putNumber("simpleReefAlignment/speeds/vxMPS", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("simpleReefAlignment/speeds/vyMPS", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("simpleReefAlignment/speeds/omegaRPS", speeds.omegaRadiansPerSecond);

    //Potential dynamic tolerance that changes based on the angle of the reef (b/c math)
    // double dynamicTolerance = (0.03 * (Math.cos(Math.toRadians(reefAngle))));
    // SmartDashboard.putNumber("simpleReefAlignment/speedToleranceMPS", dynamicTolerance);

    LoggedTunableNumber staticTolerance = new LoggedTunableNumber("simpleReefAlignment/minimumSpeedTolerance", 0.03);

    //Determine if the function is finished based on how slow the robot is going (<0.03 MPS)
    finished =
        speeds.vxMetersPerSecond <= staticTolerance.get()
            && speeds.vyMetersPerSecond <= staticTolerance.get();

    SmartDashboard.putBoolean("simpleReefAlignment/isFinished", finished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
