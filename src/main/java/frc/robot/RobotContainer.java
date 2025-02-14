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

import static frc.robot.Constants.CoralPos.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralPos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
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
  private final LimelightSubsystem limelight;
  private final ScoringSubsystem score = new ScoringSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  // Controller
  private final CommandPS5Controller base = new CommandPS5Controller(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);

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
        break;
    }
    limelight = new LimelightSubsystem();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set default commands for all subsystems
    setDefaultCommands();

    // Register named commands for Pathplanner autonomous routines
    registerNamedCommands();

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

    limelight.setDefaultCommand(limelight.setLimelight());

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

    // Reset gyro to 0° when B button is pressed
    base.options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // // base.square().onTrue(pivot.setGoal(5.0));
    // base.R1()
    //     .onTrue(
    //         new ParallelRaceGroup(
    //             elevator.setGoal(CoralPos.TEST.getElevatorSetpoint()),
    //             pivot.setGoal(CoralPos.TEST.getPivotSetpoint())));
    base.circle().onTrue(pivot.setGoal(25.0));
    base.cross().onTrue(new ParallelCommandGroup(elevator.setGoal(5), pivot.setGoal(47.5)));
    base.R2().whileTrue(score.setSpeed(-0.8)); // intaking
    base.R2().onFalse(score.setSpeed(-0.4)); // intaking

    base.L2().whileTrue(score.setSpeed(0.5)); // scores coral
    //  base.L2().onFalse(score.setSpeed(.05)); // scores coral
    base.R1().whileTrue(intake.setSpeed(0.5));
    // base.R1().onTrue(elevator.setGoal(39));
    // base.L1().onTrue(elevator.setGoal(0.5));
    // operator.cross().onTrue(pivot.setGoal(10.0));
    base.touchpad()
        .onTrue(
            new ParallelCommandGroup(
                elevator.setGoal(HOME.getElevatorSetpoint()),
                pivot.setGoal(HOME.getPivotSetpoint())));
    base.povUp()
        .onTrue(
            new ParallelCommandGroup(
                elevator.setGoal(CoralPos.LEVERLFOUR.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.LEVERLFOUR.getPivotSetpoint())));
    base.povDown()
        .onTrue(
            new ParallelCommandGroup(
                elevator.setGoal(CoralPos.LEVELTWO.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.LEVELTWO.getPivotSetpoint())));
    base.povLeft()
        .onTrue(
            new ParallelCommandGroup(
                elevator.setGoal(CoralPos.LEVELONE.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.LEVELONE.getPivotSetpoint())));
    base.povRight()
        .onTrue(
            new ParallelCommandGroup(
                elevator.setGoal(CoralPos.LEVELTHREE.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.LEVELTHREE.getPivotSetpoint())));

    base.triangle()
        .onTrue(
            new ParallelRaceGroup(
                elevator.setGoal(CoralPos.ALGAEINTAKEONE.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.ALGAEINTAKEONE.getPivotSetpoint())));

    base.circle()
        .onTrue(
            new ParallelRaceGroup(
                elevator.setGoal(CoralPos.ALGAEINTAKETWO.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.ALGAEINTAKETWO.getPivotSetpoint())));
    // base.cross()
    //     .onTrue(
    //         new ParallelRaceGroup(
    //             elevator.setGoal(CoralPos.ALGAELOLLIPOP.getElevatorSetpoint()),
    //             pivot.setGoal(CoralPos.ALGAELOLLIPOP.getPivotSetpoint())));
    base.square()
        .onTrue(
            new ParallelRaceGroup(
                elevator.setGoal(CoralPos.ALGAEGROUND.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.ALGAEGROUND.getPivotSetpoint())));
    base.L1()
        .onTrue(
            new ParallelRaceGroup(
                elevator.setGoal(CoralPos.CLIMB.getElevatorSetpoint()),
                pivot.setGoal(CoralPos.CLIMB.getPivotSetpoint())));
    // base.L3().whileTrue(DriveCommands.limelightDriveToReef(drive));
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

    score.setDefaultCommand(score.DefaultCommand());
    pivot.setDefaultCommand(pivot.resetPivot());
    elevator.setDefaultCommand(elevator.resetElevators());
    intake.setDefaultCommand(intake.DefaultCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
