// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FireFromSubwoofer;
import frc.robot.commands.JoystickArmCommand;
import frc.robot.commands.MoveArmToSafeZoneShot;
import frc.robot.commands.MoveArmToSpeakerShot;
import frc.robot.commands.RollerButtonCommand;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  public double armControlValue;
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);
  public static XboxController operatorController = new XboxController(2);
  public static XboxController driverXbox = new XboxController(0);


  private final JoystickArmCommand m_joystickArmCommand;
  private final RollerButtonCommand m_RollerButtonCommand;
  private final MoveArmToSpeakerShot m_MoveArmToSpeakerShot;
  private final MoveArmToSafeZoneShot m_MoveArmSafeShot;

  //auto commands
  private final FireFromSubwoofer m_fireFromSubwoofer;
  private final AutoIntake m_autoIntake;

  private final SendableChooser<Command> autoChooser;


  //public double speedCorrectionFactor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    m_joystickArmCommand = new JoystickArmCommand(m_arm, operatorController);  //control arm manually with joysticks
    m_RollerButtonCommand = new RollerButtonCommand(m_shooter, m_intake, driverXbox, operatorController); //control all rollers with buttons

    m_intake.setDefaultCommand(m_RollerButtonCommand);
    m_shooter.setDefaultCommand(m_RollerButtonCommand);
    m_arm.setDefaultCommand(m_joystickArmCommand);

    m_MoveArmToSpeakerShot = new MoveArmToSpeakerShot(m_arm, operatorController);
    m_MoveArmSafeShot = new MoveArmToSafeZoneShot(m_arm, operatorController);

    // Register Named Commands
    //Named commands = commands other than driving around that still need to be executed in auto
    m_fireFromSubwoofer = new FireFromSubwoofer(m_arm, m_shooter);
    m_autoIntake = new AutoIntake(m_intake, m_shooter, m_arm);

    NamedCommands.registerCommand("Fire From Subwoofer", m_fireFromSubwoofer);
    NamedCommands.registerCommand("Run Intake", m_autoIntake);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
        () -> -driverXbox.getRightX());

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

    new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    //new JoystickButton(operatorController, 1).onTrue(m_autoAmpSequence);
    new JoystickButton(operatorController, 3).onTrue(m_MoveArmToSpeakerShot);
    new JoystickButton(operatorController, 2).onTrue(m_MoveArmSafeShot);


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
    // An example command will be run in autonomous
    return autoChooser.getSelected();
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
