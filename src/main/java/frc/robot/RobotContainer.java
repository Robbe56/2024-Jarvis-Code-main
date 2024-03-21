// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.AutoAmpDrive;
import frc.robot.commands.swervedrive.auto.AutoModeAimAndFire;
import frc.robot.commands.swervedrive.auto.FireFromMidline;
import frc.robot.commands.swervedrive.auto.MidlineTurnCommand;
import frc.robot.commands.swervedrive.auto.MidlineTurnCommandRed;
import frc.robot.commands.swervedrive.auto.MidlineUnturnCommand;
import frc.robot.commands.swervedrive.auto.MidlineUnturnCommandRed;
import frc.robot.commands.swervedrive.auto.TargetNoteCommand;
import frc.robot.commands.swervedrive.auto.TargetNoteCommandTeleop;
import frc.robot.commands.swervedrive.auto.TargetNoteCrossFieldCommand;
import frc.robot.commands.swervedrive.auto.TurnToSpeaker;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ArmDownAutoCommand;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FireFromDistance;
import frc.robot.commands.FireFromSubwoofer;
import frc.robot.commands.FirePreparedShot;
import frc.robot.commands.HangOnChainCommand;
import frc.robot.commands.JoystickArmCommand;
import frc.robot.commands.MoveArmAutoTarget;
import frc.robot.commands.MoveArmToSafeZoneShot;
import frc.robot.commands.MoveArmToSpeakerShot;
import frc.robot.commands.PrepareToFire;
import frc.robot.commands.RollerButtonCommand;
import frc.robot.commands.ShootAcrossFieldCommand;
import frc.robot.commands.UnwindHangerCommand;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Subsystems
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final HangSubsystem m_hang = new HangSubsystem();

  public double armControlValue;
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // Joysticks
  public static XboxController operatorController = new XboxController(2);
  public static XboxController driverXbox = new XboxController(0);

  //Auto Mode Chooser
  //private final SendableChooser<Command> autoChooser;

  //Commands
  private final JoystickArmCommand m_joystickArmCommand;
  private final RollerButtonCommand m_RollerButtonCommand;
  private final HangOnChainCommand m_hangCommand;
  private final UnwindHangerCommand m_unwind;
  private final MoveArmToSpeakerShot m_MoveArmToSpeakerShot;
  private final MoveArmToSafeZoneShot m_MoveArmSafeShot;
  private final TargetNoteCommand m_findNote;
  private final TargetNoteCommandTeleop m_findNoteTeleop;
  private final TurnToSpeaker m_turnToSpeaker;
  private final ShootAcrossFieldCommand m_acrossFieldShot;
  private final TargetNoteCrossFieldCommand m_targetMidline;
  private final MidlineTurnCommand m_midlineTurn;
  private final MidlineUnturnCommand m_unturn;
  private final MidlineTurnCommandRed m_midlineTurnRed;
  private final MidlineUnturnCommandRed m_unturnRed;
  private final FireFromMidline m_fireFromMidline;
  private final MoveArmAutoTarget m_autoArm;
  private final AutoAmpDrive m_autoAmpDrive;
  private final AutoModeAimAndFire m_autoAimFire;
  private final ArmDownAutoCommand m_armDown;


  private final SendableChooser<Command> autoChooser;

  //auto commands
  //private final FireFromSubwoofer m_fireFromSubwoofer;
  //private final FireFromDistance m_fireFromDistance;
  //private final AutoIntake m_autoIntake;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    //PathPlanner Named Commands
    NamedCommands.registerCommand("Fire From Subwoofer", new FireFromSubwoofer(m_arm, m_shooter));
    NamedCommands.registerCommand("Fire From Distance", new FireFromDistance(m_arm, m_shooter, m_intake));
    NamedCommands.registerCommand("Run Intake", new AutoIntake(m_intake, m_shooter, m_arm));
    NamedCommands.registerCommand("Auto Target Note", new TargetNoteCommand(drivebase, m_intake, m_shooter, m_arm));
    NamedCommands.registerCommand("Shoot Across Field", new ShootAcrossFieldCommand(m_arm, m_shooter, m_intake));
    NamedCommands.registerCommand("Target Note At Midline", new TargetNoteCrossFieldCommand(drivebase, m_intake, m_shooter, m_arm));
    NamedCommands.registerCommand("Prepare To Fire", new PrepareToFire(m_arm, m_shooter, m_intake));
    NamedCommands.registerCommand("Fire Prepared Shot", new FirePreparedShot(m_arm, m_shooter, m_intake));
    NamedCommands.registerCommand("Fire From Midline", new FireFromMidline(m_arm, m_shooter, m_intake));
    NamedCommands.registerCommand("Midline Turn Command", new MidlineTurnCommand(drivebase));
    NamedCommands.registerCommand("Midline Aim At Wall", new MidlineUnturnCommand(drivebase));
    NamedCommands.registerCommand("Midline Turn Command - Red", new MidlineTurnCommandRed(drivebase));
    NamedCommands.registerCommand("Midline Aim At Wall - Red", new MidlineUnturnCommandRed(drivebase));
    NamedCommands.registerCommand("Auto Aim And Fire", new AutoModeAimAndFire(drivebase, m_arm, m_shooter));
    NamedCommands.registerCommand("Arm Down", new ArmDownAutoCommand(m_arm));

    m_joystickArmCommand = new JoystickArmCommand(m_arm, operatorController);  //control arm manually with joysticks
    m_RollerButtonCommand = new RollerButtonCommand(m_shooter, m_intake, m_arm, driverXbox, operatorController); //control all rollers with buttons

    m_intake.setDefaultCommand(m_RollerButtonCommand);
    m_shooter.setDefaultCommand(m_RollerButtonCommand);
    m_arm.setDefaultCommand(m_joystickArmCommand);

    m_MoveArmToSpeakerShot = new MoveArmToSpeakerShot(m_arm, operatorController);
    m_MoveArmSafeShot = new MoveArmToSafeZoneShot(m_arm, operatorController);
    m_hangCommand = new HangOnChainCommand(m_hang, m_arm, operatorController);
    m_unwind = new UnwindHangerCommand(m_hang, m_arm, operatorController);
    m_findNote = new TargetNoteCommand(drivebase, m_intake, m_shooter, m_arm);
    m_findNoteTeleop = new TargetNoteCommandTeleop(drivebase, m_intake, m_shooter, m_arm, driverXbox);
    m_turnToSpeaker = new TurnToSpeaker(drivebase, driverXbox);
    m_acrossFieldShot = new ShootAcrossFieldCommand(m_arm, m_shooter, m_intake);
    m_targetMidline = new TargetNoteCrossFieldCommand(drivebase, m_intake, m_shooter, m_arm);
    m_fireFromMidline = new FireFromMidline(m_arm, m_shooter, m_intake);
    m_autoArm = new MoveArmAutoTarget(m_arm, drivebase, operatorController);
    m_autoAmpDrive = new AutoAmpDrive(drivebase, driverXbox);
    m_autoAimFire = new AutoModeAimAndFire(drivebase, m_arm, m_shooter);
    m_armDown = new ArmDownAutoCommand(m_arm);

    m_midlineTurn = new MidlineTurnCommand(drivebase);
    m_unturn = new MidlineUnturnCommand(drivebase);
    m_midlineTurnRed = new MidlineTurnCommandRed(drivebase);
    m_unturnRed = new MidlineUnturnCommandRed(drivebase);
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
   
/*
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
*/
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(Constants.Drivebase.SlowDownTurn*-driverXbox.getRightX(), OperatorConstants.RotationDeadband));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
 //       drivebase.setDefaultCommand(
 //       !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 8).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driverXbox, 1).onTrue(m_turnToSpeaker);
    new JoystickButton(driverXbox, 2).onTrue(m_findNoteTeleop);
    new JoystickButton(driverXbox, 3).onTrue(m_autoAmpDrive);
    //new JoystickButton(driverXbox, 1).onTrue(m_turnAndDetect);
    //new JoystickButton(driverXbox, 3).onTrue(m_turnToSpeaker);

    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    //new JoystickButton(operatorController, 1).onTrue(m_autoAmpSequence);
    new JoystickButton(operatorController, 3).onTrue(m_MoveArmToSpeakerShot);
    new JoystickButton(operatorController, 2).onTrue(m_MoveArmSafeShot);
    new JoystickButton(operatorController, 1).onTrue(m_autoArm);

    new JoystickButton(operatorController, 8).onTrue(m_unwind);
    new JoystickButton(operatorController, 10).onTrue(m_hangCommand);


    //new JoystickButton(operatorController, 6).onTrue(new InstantCommand(m_shooter::FeedMotorFast));
    //new JoystickButton(operatorController, 6).onFalse(new InstantCommand(m_shooter::StopFeedRoller));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {    
    //AMP SIDE AUTOMODES
    return new PathPlannerAuto("Amp - Auto Track");

    //CENTER AUTOMODES
    //return new PathPlannerAuto("Center - Under Stage");
    //return new PathPlannerAuto("Center Prototype - 4 Notes");
    //return new PathPlannerAuto("Center - 4 Notes");
   
    //SOURCE AUTOMODES
    //return new PathPlannerAuto("Source - Midline Robbery - Blue");
    //return new PathPlannerAuto("Source - Midline Robbery - Red");
    //return new PathPlannerAuto("Source - Get 2 Far Notes");
    //return new PathPlannerAuto("Source - Get Close Note - Get Far Note");
    //return new PathPlannerAuto("Source - Score One and Hide");
 

    //return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
