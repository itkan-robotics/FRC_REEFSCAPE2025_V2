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
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.DisabledInstantCommand;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Initialize Subsystems
  private final Drive drive;
  public static final FullArmSubsystem fullArm = new FullArmSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final AutoScoreSelection storedState = new AutoScoreSelection();

  // Initialize Controllers

  /**
   * Controller for our driver. Controls scoring L1-L4, auto-alignment, algae (in/out)take, and
   * robot movement.
   */
  private final CommandPS5Controller baseCommand = new CommandPS5Controller(0);

  private final PS5Controller base = baseCommand.getHID();
  /**
   * Controller for our operator. Controls auto-alignment coral level, left/right branch, and
   * outtake after auto-align is completed.
   */
  private final CommandPS5Controller operatorCommand = new CommandPS5Controller(1);

  private final PS5Controller operator = operatorCommand.getHID();

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
    baseCommand
        .options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    baseCommand.triangle().onTrue(fullArm.setGoal(L4, true));
    baseCommand.square().onTrue(fullArm.setGoal(L3, true));
    baseCommand.circle().onTrue(fullArm.setGoal(L2, true));
    baseCommand.touchpad().onTrue(fullArm.setGoal(L1, false));
    baseCommand.cross().onTrue(fullArm.setGoal(HOME, false));

    baseCommand.R2().whileTrue(new SmartIntake(intake, fullArm));
    // .onFalse(intake.setIntakeSpeed(-0.3).withTimeout(0.035));
    // .onFalse(fullArm.setGoal(HOME, false));
    baseCommand.R1().whileTrue(intake.setIntakeSpeed(-1));

    operatorCommand.R2().whileTrue(intake.outtakeBotState(storedState));

    operatorCommand.povDown().onTrue(fullArm.setGoal(CLIMB, true));
    operatorCommand.povUp().onTrue(fullArm.setGoal(PRECLIMB, false));

    baseCommand
        .povUp()
        .onTrue(fullArm.setGoal(HIGHALGAE, true).alongWith(intake.setIntakeSpeed(1)));
    baseCommand
        .povDown()
        .onTrue(fullArm.setGoal(LOWALGAE, true).alongWith(intake.setIntakeSpeed(-1)));

    baseCommand.L2().whileTrue(new SmartAlignProfiledPID(drive, fullArm, storedState));

    // Auto Turn to Reef Face
    baseCommand
        .R3()
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  storedState.invertAutoTurn();
                }));

    (operatorCommand.L2())
        .and(operatorCommand.triangle())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L4);
                }));

    (operatorCommand.L2())
        .and(operatorCommand.circle())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L3);
                }));

    (operatorCommand.L2())
        .and(operatorCommand.square())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L2);
                }));

    (operatorCommand.L2())
        .and(operatorCommand.touchpad())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setBotState(L1);
                }));

    (operatorCommand.L2())
        .and(operatorCommand.L1())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setLimelightPipeLine("LEFT");
                }));

    (operatorCommand.L2())
        .and(operatorCommand.R1())
        .onTrue(
            new InstantCommand(
                () -> {
                  storedState.setLimelightPipeLine("RIGHT");
                }));

    operatorCommand
        .L2()
        .whileTrue(
            new RepeatCommand(
                new DeferredCommand(
                    () ->
                        new InstantCommand(
                            () -> {
                              storedState.setOperatorReefAngle(
                                  () -> -operatorCommand.getRightY(),
                                  () -> operatorCommand.getRightX());
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

    operatorCommand
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

    NamedCommands.registerCommand(
        "goToReef", new AutoSmartAlignProfiledPID3d(drive, LimelightConstants.leftLimelightName));

    NamedCommands.registerCommand(
        "goToReefRight",
        new AutoSmartAlignProfiledPID3d(drive, LimelightConstants.rightLimelightName));

    NamedCommands.registerCommand("L4", fullArm.setGoal(L4, true));

    NamedCommands.registerCommand("PREP_L4", fullArm.setGoal(PREP_L4, true));

    NamedCommands.registerCommand("L3", fullArm.setGoal(L3, true));

    NamedCommands.registerCommand("HOME", fullArm.setGoal(HOME, false));

    NamedCommands.registerCommand("INTAKE", fullArm.setGoal(INTAKE, false));

    NamedCommands.registerCommand("INTAKEARM", fullArm.setGoal(INTAKEARM, true));

    NamedCommands.registerCommand("intakeDefault", intake.setIntakeSpeed(0.2));

    NamedCommands.registerCommand("outtake", intake.setIntakeSpeed(-0.7));

    NamedCommands.registerCommand("intake", intake.setIntakeSpeed(0.1 * 10));
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
