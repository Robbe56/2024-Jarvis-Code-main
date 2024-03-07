// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurnToSpeaker extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final XboxController driverController;
  private final Timer timer;
  private final ChassisSpeeds autoTrackSpeakerSpeeds;
  private double turnSpeed;

  public TurnToSpeaker(SwerveSubsystem m_swerveDrive, XboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    driverController = m_driverController;
    timer = new Timer();

    autoTrackSpeakerSpeeds = new ChassisSpeeds(0,0, turnSpeed);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnSpeed = Constants.Drivebase.SpeakerTrackKP*swerveDrive.TrackSpeaker(); //multiply Limelight value by P factor
    swerveDrive.drive(autoTrackSpeakerSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Shooter.TrackNoteTime || !driverController.getRawButton(2); //go back to regular driving after 5 seconds or when you let go of button #2

  }
}
