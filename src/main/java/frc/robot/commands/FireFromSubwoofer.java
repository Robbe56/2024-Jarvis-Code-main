// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireFromSubwoofer extends Command {
  /** Creates a new FireFromSubwoofer. */
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  private final Timer timer;

  
  public FireFromSubwoofer(ArmSubsystem m_arm, ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    shooter = m_shooter;
    timer = new Timer();

    addRequirements(arm, shooter);
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
    
    // arm contol
    if (timer.get() < Constants.Shooter.NoteInAirTime){
      if (arm.GetArmEncoderPosition() < Constants.Shooter.aimedAtSpeaker){
        arm.ArmUpCommand();
    }
      else {arm.ArmHoldPosition();}

  } else arm.ArmDownCommand();
    
    //shooter control
    if (timer.get() < Constants.Shooter.ShooterSpinUpTime){
      shooter.ShooterIntoSpeakerSpeed();
      shooter.StopFeedRoller();
    }
    else {
      shooter.ShooterIntoSpeakerSpeed();
      shooter.FeedMotorFast();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.StopFeedRoller();
    shooter.StopShooter();
    arm.StopArm();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > Constants.Shooter.NoteInAirTime && !arm.GetBottomLimitSwitch()); //shots have been fired and arm is back down
  }
}
