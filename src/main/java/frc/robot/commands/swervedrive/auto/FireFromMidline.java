// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FireFromMidline extends Command {
  /** Creates a new FireFromSubwoofer. */
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  private final SwerveSubsystem drivebase;
  private final Timer timer;

  private final ChassisSpeeds chassisSpeeds;

  
  public FireFromMidline(ArmSubsystem m_arm, SwerveSubsystem m_drivebase, ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    //Shooting from the midline back across the field
    arm = m_arm;
    shooter = m_shooter;
    drivebase = m_drivebase;
    timer = new Timer();

    chassisSpeeds = new ChassisSpeeds(0,0, 0);

    addRequirements(arm, shooter, drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    arm.ResetArmEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

  //move arm to correct postion 
  if (arm.GetArmEncoderPosition() < Constants.Shooter.aimedAtSpeaker){
  arm.ArmUpCommand();
  }
  else arm.ArmHoldPosition();

  
  //shooter wheel control
  if (timer.get() < Constants.Shooter.WaitForArm){
    shooter.StopFeedRoller();
    shooter.StopShooter();
  }
  
  if (timer.get() > Constants.Shooter.WaitForArm && timer.get() < (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime)){
    shooter.FeedMotorsBackward();
    shooter.ShooterMotorsBackward();
  }

  if (timer.get() > (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime) && timer.get() < (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.AcrossFieldSpinUpTime)){
    shooter.StopFeedRoller();
    shooter.ShooterIntoAmpSpeed();
  }

  if (timer.get() > (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.AcrossFieldSpinUpTime) && timer.get() < (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.AcrossFieldSpinUpTime + Constants.Shooter.MidlineNoteInAir)){
    shooter.FeedMotorFast();
    shooter.ShooterIntoAmpSpeed();
  }
  drivebase.drive(chassisSpeeds);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    shooter.StopFeedRoller();
    shooter.StopShooter();
    arm.StopArm();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.AcrossFieldSpinUpTime + Constants.Shooter.MidlineNoteInAir);

  }
}