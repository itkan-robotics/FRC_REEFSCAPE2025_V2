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

import static frc.robot.Constants.BotState.*;

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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StateMachineCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
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
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ScoringSubsystem score = new ScoringSubsystem();
  private final ActuatorSubsystem actuators = new ActuatorSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final StateMachine currState = new StateMachine();
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

    // Reset gyro to 0° when options button is pressed
    base.options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Command for when we are testing different positions
    // base.PS().onTrue(new PivElevStateCommand(elevator, pivot, TEST));

    // Intaking coral
    base.R2()
        .or(operator.R2())
        .whileTrue(
            score
                .setSpeed(0)
                .alongWith(intake.setSpeed(0.5))
                .alongWith(new StateMachineCommand(elevator, actuators, currState, HOME)));
    // Scoring Coral
    base.L2().or(operator.L2()).whileTrue(score.setSpeed(-0.15));

    // Intaking algae
    // base.L2().whileTrue(score.intakeAlgae(-0.5)).onFalse(score.intakeAlgae(-0.2));
    base.cross()
        .or(operator.cross())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, HOME));

    base.create()
        .onTrue(
            new StateMachineCommand(elevator, actuators, currState, GROUNDALGAE)
                .alongWith(score.intakeAlgae(-0.5)));
    // base.create().onFalse(score.intakeAlgae(-0.2));
    // Coral Positioning Commands
    base.triangle()
        .or(operator.triangle())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L4));
    base.touchpad()
        .or(operator.touchpad())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L1));
    base.square()
        .or(operator.square())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L2));
    base.circle()
        .or(operator.circle())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L3));

    base.povDown()
        .or(operator.povDown())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, LOWALGAE));
    base.povUp()
        .or(operator.povUp())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, HIGHALGAE));
    base.povLeft()
        .or(operator.povLeft())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, RESET));
    base.povRight()
        .or(operator.povRight())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, CLIMB));

    //  Reef Alignment Commands
    //     base.PS().whileTrue(new ReefAlignmentCommand(drive, limelight, ReefSide.RIGHT));
    //     base.L2().whileTrue(new ReefAlignmentCommand(drive, limelight, null));
    //     base.create().whileTrue(new ReefAlignmentCommand(drive, limelight, ReefSide.LEFT));
    //    base.create().onTrue(new StateMachineCommand(elevator, actuators, currState, RESET));

    // Algae Positioning Commands
    // base.triangle().onTrue(new PivElevStateCommand(elevator, pivot, LOWALGAE));
    // base.circle().onTrue(new PivElevStateCommand(elevator, pivot, HIGHALGAE));
    // base.cross().onTrue(new PivElevStateCommand(elevator, pivot, LOLLIPOPALGAE));
    // base.square().onTrue(new PivElevStateCommand(elevator, pivot, GROUNDALGAE));
    // base.triangle()
    //     .and(base.square())
    //     .and(base.circle())
    //     .and(base.cross())
    //     .onTrue(new StateMachineCommand(elevator, actuators, currState, CLIMB));

    // Reef Alignment Commands
    // base.PS().whileTrue(new ReefAlignmentCommand(drive, limelight, ReefSide.RIGHT));
    // base.L2().whileTrue(new ReefAlignmentCommand(drive, limelight, null));
    // base.create().whileTrue(new ReefAlignmentCommand(drive, limelight, ReefSide.LEFT));
  }

  /*********************************************************
   * Use this to set up PathplannerLib Named Commands
   * for autonomous routines.
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

    NamedCommands.registerCommand("intake", intake.setSpeed(0.5));

    NamedCommands.registerCommand("stopIntake", intake.DefaultCommand());

    NamedCommands.registerCommand(
        "pivElevL4", new StateMachineCommand(elevator, actuators, currState, L4));

    NamedCommands.registerCommand(
        "pivElevHome", new StateMachineCommand(elevator, actuators, currState, HOME));
  }

  /*********************************************************
   * Use this to set up default commands for subsystems.
   *********************************************************/

  public void setDefaultCommands() {
    // Default command, field-centric drive & field-centric angle
    drive.setDefaultCommand(
        DriveCommands.joystickMDrive(
            drive,
            () -> -base.getLeftY() * elevator.getSlowDownMult(),
            () -> -base.getLeftX() * elevator.getSlowDownMult(),
            () -> -base.getRightY() * elevator.getSlowDownMult(),
            () -> base.getRightX() * elevator.getSlowDownMult()));

    // Set the robot to the RESET position
    currState.setState(RESET);
    actuators.setGoal(currState.getState().getPivotSetpoint());
    elevator.setGoal(currState.getState().getElevatorSetpoint());

    score.setDefaultCommand(score.DefaultCommand());
    intake.setDefaultCommand(intake.DefaultCommand());
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
}
