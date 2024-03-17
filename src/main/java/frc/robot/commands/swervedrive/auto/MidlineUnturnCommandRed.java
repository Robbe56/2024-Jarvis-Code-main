// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MidlineUnturnCommandRed extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final Timer timer;
  private final ChassisSpeeds midlineTurnChassisSpeeds;

  public MidlineUnturnCommandRed(SwerveSubsystem m_swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    timer = new Timer();

    midlineTurnChassisSpeeds = new ChassisSpeeds(0,0, 0);

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
    if (swerveDrive.getHeading().getDegrees() < -Constants.Auton.LookAcrossFieldAngle){
      
    midlineTurnChassisSpeeds.omegaRadiansPerSecond = Constants.Auton.midlineTurnSpeed;
    }
    else midlineTurnChassisSpeeds.omegaRadiansPerSecond = 0;
    
    swerveDrive.drive(midlineTurnChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveDrive.getHeading().getDegrees() > -Constants.Auton.LookAcrossFieldAngle; //end command when robot has turned to the target

  

  }
}
