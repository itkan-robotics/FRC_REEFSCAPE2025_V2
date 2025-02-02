// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.LimelightConstants.kReefOffset;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TagOffsets;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.limelightReefAlignment;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralOuttake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final LimelightSubsystem limelight;
  private final CoralIntake intake = new CoralIntake();
  private final CoralOuttake outtake = new CoralOuttake();
  // Controller
  private final CommandPS5Controller base = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new ElevatorSubsystem();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        elevator = new ElevatorSubsystem();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new ElevatorSubsystem();
        break;
    }
    limelight = new LimelightSubsystem();

    // Set default commands for all subsystems
    setDefaultCommands();

    // Register named commands for Pathplanner autonomous routines
    registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId Routines
    setUpSysIdRoutines(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -base.getLeftY(),
    //             () -> -base.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // base.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when options button is pressed
    base.options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    base.square().onTrue(elevator.setGoal(5.0));
    base.triangle().onTrue(elevator.setGoal(15.0));
    base.circle().onTrue(elevator.setGoal(10.0));
    base.cross().onTrue(elevator.setGoal(0.0));
    base.R2().whileTrue(intake.Intake(0.4));
    base.L2().whileTrue(outtake.Outtake(0.25));

    base.povUp()
        .whileTrue(new limelightReefAlignment(drive, limelight, kReefOffset, TagOffsets.CENTER));
    base.povLeft()
        .whileTrue(
            new limelightReefAlignment(drive, limelight, kReefOffset, TagOffsets.LEFT_BRANCH));
    base.povRight()
        .whileTrue(
            new limelightReefAlignment(drive, limelight, kReefOffset, TagOffsets.RIGHT_BRANCH));

    // Sets the robot's believed position to (Right against the reef, ID = 19)
    // PLEASE DISABLE/REMOVE BEFORE AND DURING MATCHES
    if (base.options().getAsBoolean()) {
      base.povDown()
          .onTrue(drive.setPoseCommand(new Pose2d(3.65, 5.4, Rotation2d.fromDegrees(-60))));
      // ID = 21: new Pose2d(6.162, 4.020, Rotation2d.fromDegrees(180)
    }
  }

  /*********************************************************
   * Use this to set up PathplannerLib Named Commands
   * for autonomous routines.
   * <p> Last Updated by Abdullah Khaled, 1/18/2025
   *********************************************************/

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "setGyroTo180",
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(), new Rotation2d(Math.toRadians(180)))),
                drive)
            .ignoringDisable(true));
  }

  /*********************************************************
   * Use this to set up default commands for subsystems.
   * <p> Last Updated by Abdullah Khaled, 1/18/2025
   *********************************************************/

  public void setDefaultCommands() {
    // Default command, field-centric drive & field-centric angle
    drive.setDefaultCommand(
        DriveCommands.joystickMDrive(
            drive,
            () -> -base.getLeftY(),
            () -> -base.getLeftX(),
            () -> -base.getRightY(),
            () -> base.getRightX()));

    intake.setDefaultCommand(intake.DefaultCommand());
    outtake.setDefaultCommand(outtake.DefaultCommand());
    limelight.setDefaultCommand(limelight.setLimelight());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /*********************************************************
   * Used to set up 6328's SysId routines
   * <p> Last Updated by Abdullah Khaled, 2/1/2025
   *********************************************************/
  public void setUpSysIdRoutines(LoggedDashboardChooser<Command> m_autoChooser) {
    // Set up SysId routines
    m_autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    m_autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
