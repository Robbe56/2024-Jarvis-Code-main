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

public class AutoModeAimAndFire extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  private final Timer timer;
  private final ChassisSpeeds autoTrackSpeakerSpeeds;

  public AutoModeAimAndFire(SwerveSubsystem m_swerveDrive, ArmSubsystem m_arm, ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    arm = m_arm;
    shooter = m_shooter;
    timer = new Timer();

    autoTrackSpeakerSpeeds = new ChassisSpeeds(0,0, 0);

    addRequirements(swerveDrive, arm);
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
    //move robot off of subwoofer
    if (timer.get() < Constants.Shooter.MoveOffSubWoofer){
      autoTrackSpeakerSpeeds.vxMetersPerSecond = Constants.Drivebase.autoForwardSpeed;
      autoTrackSpeakerSpeeds.omegaRadiansPerSecond = 0;
      
      arm.StopArm();
    }
    //stop moving backwards and rotate robot to align with apriltag
    if (timer.get() > Constants.Shooter.MoveOffSubWoofer){
      autoTrackSpeakerSpeeds.vxMetersPerSecond = 0;
      autoTrackSpeakerSpeeds.omegaRadiansPerSecond = Constants.Drivebase.SpeakerTrackKP*swerveDrive.TrackSpeaker(); //multiply Limelight value by P factor
      
      //move arm to correct postion based on apriltag
      if (arm.GetArmEncoderPosition() < Constants.Shooter.armEquationSlope*swerveDrive.TrackSpeakerHeight() + Constants.Shooter.armEquationIntercept){
      arm.ArmUpCommand();
      }
      else arm.ArmHoldPosition();
    }
  
    swerveDrive.drive(autoTrackSpeakerSpeeds);
  


  //shooter wheel control
  if (timer.get() < Constants.Shooter.WaitForArm){
    shooter.StopFeedRoller();
    shooter.StopShooter();
  }
  
  if (timer.get() > Constants.Shooter.WaitForArm && timer.get() < (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime)){
    shooter.FeedMotorsBackward();
    shooter.ShooterMotorsBackward();
  }

  if (timer.get() > (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime) && timer.get() < (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.ShooterSpinUpTime)){
    shooter.StopFeedRoller();
    shooter.ShooterIntoSpeakerSpeed();
  }

  if (timer.get() > (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.ShooterSpinUpTime) && timer.get() < (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.ShooterSpinUpTime + Constants.Shooter.AutoModeNoteInAir)){
    shooter.FeedMotorFast();
    shooter.ShooterIntoSpeakerSpeed();
  }
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
    return timer.get() > (Constants.Shooter.WaitForArm + Constants.Shooter.autoModeUnjamTime + Constants.Shooter.ShooterSpinUpTime + Constants.Shooter.AutoModeNoteInAir);

  }
}
