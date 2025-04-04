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

import static frc.robot.util.MachineStates.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.AutoSmartAlignProfiledPID3d;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SmartAlignProfiledPID;
import frc.robot.commands.SmartIntake;
import frc.robot.subsystems.FullArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.DisabledInstantCommand;
import frc.robot.util.MachineStates;
import frc.robot.util.MachineStates.BotState;
import java.util.Set;
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
  public static final FullArmSubsystem fullArm = new FullArmSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();

  private static final LimelightSubsystem limelight =
      new LimelightSubsystem(LimelightConstants.leftLimelightName);

  private final BotState currState = MachineStates.RESET;
  public static final AutoScoreSelection storedState = new AutoScoreSelection();

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

    // Register named commands for Pathplanner autonomous routines
    registerNamedCommands();

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

    base.triangle().onTrue(fullArm.setGoal(L4, true));
    base.square().onTrue(fullArm.setGoal(L3, true));
    base.circle().onTrue(fullArm.setGoal(L2, true));
    base.touchpad().onTrue(fullArm.setGoal(L1, false));
    base.cross().onTrue(fullArm.setGoal(HOME, false));

    base.R2().whileTrue(new SmartIntake(intake, fullArm));
    //     .onFalse(intake.setIntakeSpeed(-0.3).withTimeout(0.035));
    // .onFalse(fullArm.setGoal(HOME, false));h
    base.R1().whileTrue(intake.setIntakeSpeed(-1));

    operator.R2().whileTrue(intake.setIntakeSpeed(-0.42069));

    operator.povDown().onTrue(fullArm.setGoal(CLIMB, true));
    operator.povUp().onTrue(fullArm.setGoal(PRECLIMB, false));

    base.povUp().onTrue(fullArm.setGoal(HIGHALGAE, true).alongWith(intake.setIntakeSpeed(1)));
    base.povDown().onTrue(fullArm.setGoal(LOWALGAE, true).alongWith(intake.setIntakeSpeed(-1)));

    base.L2().whileTrue(new SmartAlignProfiledPID(drive, fullArm, storedState));

    // Auto Turn to Reef Face
    base.R3()
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  storedState.invertAutoTurn();
                }));

    (operator.L2())
        .and(operator.triangle())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L4);
                }));
    (operator.L2())
        .and(operator.circle())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L3);
                }));
    (operator.L2())
        .and(operator.square())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L2);
                }));
    (operator.L2())
        .and(operator.touchpad())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L1);
                }));
    (operator.L2())
        .and(operator.L1())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setLimelightPipeLine("LEFT");
                }));
    (operator.L2())
        .and(operator.R1())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setLimelightPipeLine("RIGHT");
                }));

    operator
        .L2()
        .whileTrue(
            new RepeatCommand(
                new DeferredCommand(
                    () ->
                        new InstantCommand(
                            () -> {
                              storedState.setOperatorReefAngle(
                                  () -> -operator.getRightY(), () -> operator.getRightX());
                            }),
                    Set.of())));

    // operator
    //     .PS()
    //     .onTrue(
    //         new DisabledInstantCommand(
    //             () -> {
    //               if (DriverStation.isDisabled()) {
    //                 fullArm.setCoastMode();
    //               }
    //             }));

    operator
        .PS()
        .onTrue(
            new DisabledInstantCommand(
                () -> {
                  if (DriverStation.isDisabled()) {
                    fullArm.setBrakeMode();
                  }
                }));
  }

  /*********************************************************
   * Use this to set up PathplannerLib Named Commands
   * for autonomous routines.
   *********************************************************/

  public void registerNamedCommands() {

    // NamedCommands.registerCommand(
    //     "setGyroTo180",
    //     Commands.runOnce(
    //             () ->
    //                 drive.setPose(
    //                     new Pose2d(
    //                         drive.getPose().getTranslation(), new
    // Rotation2d(Math.toRadians(180)))),
    //             drive)
    //         .ignoringDisable(true));

    // NamedCommands.registerCommand("intake", score.setSpeed(0.5));

    // NamedCommands.registerCommand("outtake", score.setSpeedAndState(-0.8, false));

    // NamedCommands.registerCommand("stopIntake", score.getDefaultCommand());

    // NamedCommands.registerCommand("stopOuttake", score.DefaultCommand());

    NamedCommands.registerCommand(
        "goToReef", new AutoSmartAlignProfiledPID3d(drive, LimelightConstants.leftLimelightName));

    NamedCommands.registerCommand(
        "goToReefRight",
        new AutoSmartAlignProfiledPID3d(drive, LimelightConstants.rightLimelightName));

    NamedCommands.registerCommand("L4", fullArm.setGoal(L4, true));

    NamedCommands.registerCommand("L3", fullArm.setGoal(L3, true));

    NamedCommands.registerCommand("HOME", fullArm.setGoal(HOME, false));

    NamedCommands.registerCommand("INTAKE", fullArm.setGoal(INTAKE, false));

    NamedCommands.registerCommand("INTAKEARM", fullArm.setGoal(INTAKEARM, true));

    NamedCommands.registerCommand("intakeDefault", intake.setIntakeSpeed(0.2));

    NamedCommands.registerCommand("outtake", intake.setIntakeSpeed(-0.7));

    NamedCommands.registerCommand("intake", intake.setIntakeSpeed(1));

    // NamedCommands.registerCommand(
    //     "CoralIntakePos", ArmCommands.setArmGoal(shoulder, extension, wrist, currState, INTAKE));

    // NamedCommands.registerCommand("outtakeDefault", score.setSpeedAndState(0.0075, false));
  }

  /*********************************************************
   * Use this to set up default commands for subsystems.
   *********************************************************/

  public void setDefaultCommands() {
    // Default command, field-centric drive & field-centric angle
    drive.setDefaultCommand(
        DriveCommands.joystickMDrive(
            drive,
            storedState,
            () -> -base.getLeftY(),
            () -> -base.getLeftX(),
            () -> -base.getRightY(),
            () -> base.getRightX(),
            () -> 1));

    // score.setDefaultCommand(score.setSpeedAndState(0.0, false));
    // limelight.setDefaultCommand(limelight.setLimelight());
    // shoulder.setDefaultCommand(shoulder.setGoal(0));
    // extension.setDefaultCommand(extension.setGoal(0));
    intake.setDefaultCommand(intake.DefaultCommand());
    fullArm.setGoal(HOME, false);
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
