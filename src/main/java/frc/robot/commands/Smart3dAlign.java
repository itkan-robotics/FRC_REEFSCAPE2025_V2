package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.TuneableProfiledPID;
import java.util.function.DoubleSupplier;

public class Smart3dAlign extends Command {

  private static final double STOP_Y_OFFSET = 0; // 0.1143; // 4.5 inches in meters

  private final Drive drive;
  private final DoubleSupplier ySupplier;
  private final TuneableProfiledPID angleController;
  private final TuneableProfiledPID alignController;

  public Smart3dAlign(Drive drive, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.ySupplier = ySupplier;
    addRequirements(drive);

    // Initialize angleController (for robot orientation)
    angleController = new TuneableProfiledPID("angleController", 2, 0.0, 0, 3, 3);
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize alignController (for lateral offset)
    alignController = new TuneableProfiledPID("alignController", 0.6, 0.0, 0, 20, 8);
    alignController.setGoal(0);
  }

  @Override
  public void execute() {
    // Our current translation is always treated as 0,0
    Translation2d currentTranslation = new Translation2d(0, 0);

    // Retrieve target pose from the specified Limelight
    double[] targetPose =
        LimelightHelpers.getTargetPose_CameraSpace(Constants.LimelightConstants.rightLimelightName);
    if (targetPose == null || targetPose.length < 3) {
      return;
    }

    // Extract raw X, Y, and rotation from the target pose.
    // Subtract STOP_Y_OFFSET to stop short of the target.
    double tx = targetPose[0];
    double ty = targetPose[1] - STOP_Y_OFFSET;
    double rotation = targetPose[5];

    // Use tx and ty directly as our goal translation.
    Translation2d goalTranslation = new Translation2d(tx, ty);

    // Calculate distance from robot (0,0) to the goal
    Translation2d robotToGoal = currentTranslation.minus(goalTranslation);
    double distanceToGoal = robotToGoal.getNorm();

    // Calculate offset vector for lateral alignment
    Translation2d offsetVector =
        new Translation2d(alignController.calculate(distanceToGoal), 0)
            .rotateBy(robotToGoal.getAngle());

    // Use the Limelight's rotation (in degrees) to align orientation.
    Rotation2d alignmentDirection = new Rotation2d(Math.toRadians(rotation));

    // Combine user joystick input (Y only, as X=0) with offset vector.
    Translation2d linearVelocity =
        new Translation2d(0, ySupplier.getAsDouble())
            .rotateBy(alignmentDirection)
            .plus(offsetVector);

    // Calculate rotational velocity with angleController
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), alignmentDirection.getRadians());

    // Construct speeds (in chassis coordinate frame)
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);

    // Command the drive to run at these speeds
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
