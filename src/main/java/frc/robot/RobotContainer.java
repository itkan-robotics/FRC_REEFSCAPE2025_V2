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
import static frc.robot.FieldConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StateMachineCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OperatorStore;
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
  public static final OperatorStore buffer = new OperatorStore();
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
        // .or(operator.L2())
        .whileTrue(
            score
                .setSpeedAndState(0.002, false)
                .alongWith(intake.setSpeed(0.75))
                .alongWith(new StateMachineCommand(elevator, actuators, currState, CORALINTAKE)))
        .onFalse(new StateMachineCommand(elevator, actuators, currState, HOME));

    // Scoring Coral
    base.R1()
        // .or(operator.L2())
        .whileTrue(score.setSpeedAndState(-1.0, false));

    // Intaking algae
    base.L2().whileTrue(score.setSpeedAndState(-base.getL2Axis() / 4, false));

    // Outtake algae
    base.L1()
        .whileTrue(score.setSpeedAndState(-0.5, true))
        .onFalse(score.setSpeedAndState(-0.1, true));

    base.cross()
        // .or(operator.cross())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, HOME));

    // Coral Positioning Commands

    base.triangle()
        // .or(operator.triangle())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L4));

    base.touchpad()
        // .or(operator.touchpad())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L1));

    base.square()
        // .or(operator.square())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L2));

    base.circle()
        // .or(operator.circle())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, L3));

    // Algae Positioning Commands

    base.povDown()
        // .or(operator.povDown())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, LOWALGAE));

    base.povUp()
        // .or(operator.povUp())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, HIGHALGAE));

    base.PS().onTrue(new StateMachineCommand(elevator, actuators, currState, BARGE));

    base.povLeft()
        // .or(operator.povLeft())
        .onTrue(new StateMachineCommand(elevator, actuators, currState, RESET));

    base.povRight()
        // .or(operator.povRight())
        .onTrue(new InstantCommand());

    base.create()
        .onTrue(
            new AutoScoreCommand(
                drive,
                actuators,
                elevator,
                intake,
                buffer,
                () -> Constants.toBotState(buffer.getBotStateInt()),
                limelight,
                currState))
        .onFalse(new InstantCommand());

    // Matthew-Align Guided Automatically (MAGA)

    operator
        .L2()
        .and(operator.triangle())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setBotStateInt(4);
                }));
    operator
        .L2()
        .and(operator.circle())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setBotStateInt(3);
                }));
    operator
        .L2()
        .and(operator.square())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setBotStateInt(2);
                }));
    operator
        .L2()
        .and(operator.touchpad())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setBotStateInt(1);
                }));
    operator
        .L2()
        .and(operator.povLeft())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setOffsetPipeLine("LEFT");
                }));
    operator
        .L2()
        .and(operator.povRight())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setOffsetPipeLine("RIGHT");
                }));
    operator
        .L2()
        .and(operator.povUp())
        .onTrue(
            new InstantCommand(
                () -> {
                  buffer.setOffsetPipeLine("CENTER");
                }));

    operator
        .L2()
        .whileTrue(
            new InstantCommand(
                () -> {
                  buffer.setOperatorAngle(() -> -operator.getRightY(), () -> operator.getRightX());
                }));
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

    NamedCommands.registerCommand("stopIntake", intake.getDefaultCommand());

    NamedCommands.registerCommand(
        "L4", new StateMachineCommand(elevator, actuators, currState, L4));

    NamedCommands.registerCommand(
        "HOME", new StateMachineCommand(elevator, actuators, currState, HOME));
  }

  /*********************************************************
   * Use this to set up default commands for subsystems.
   *********************************************************/

  public void setDefaultCommands() {
    // Default command, field-centric drive & field-centric angle
    drive.setDefaultCommand(
        DriveCommands.joystickMDrive(
            drive,
            () -> -base.getLeftY(),
            () -> -base.getLeftX(),
            () -> -base.getRightY(),
            () -> base.getRightX(),
            () -> elevator.getSlowDownMult()));

    // Set the robot to the RESET position
    currState.setState(RESET);
    actuators.setGoal(currState.getState().getActuatorSetpoint());
    elevator.setGoal(currState.getState().getElevatorSetpoint());

    score.setDefaultCommand(score.setSpeedAndState(0.000, false));
    intake.setDefaultCommand(intake.setSpeed(0.1));
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
