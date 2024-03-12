// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAcrossFieldCommand extends Command {
  /** Creates a new FireFromSubwoofer. */
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final Timer timer;

  
  public ShootAcrossFieldCommand(ArmSubsystem m_arm, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    //Use to fire note across the field
    arm = m_arm;
    shooter = m_shooter;
    intake = m_intake;
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

    if (timer.get() < Constants.Shooter.intakeTimer){
      arm.StopArm();
      intake.intakeActive();
      shooter.StopFeedRoller();
      shooter.StopShooter();

    } 

  
    
    // arm contol
    if (timer.get() < Constants.Shooter.NoteInAirAcrossFieldTime){
      if (arm.GetArmEncoderPosition() < Constants.Shooter.aimedAcrossField){
        arm.ArmUpFastCommand();
    }
      else {arm.ArmHoldPosition();}

  } else arm.ArmDownCommand();
    
    //shooter control
    if (timer.get() < Constants.Shooter.UnJamTime){
      shooter.ShooterMotorsBackward();;
      shooter.FeedMotorsBackward();
    }

    if (timer.get() > Constants.Shooter.UnJamTime && timer.get() < Constants.Shooter.AcrossFieldSpinUpTime){
      shooter.ShooterIntoSpeakerSpeed();
      shooter.StopFeedRoller();
    }

    if (timer.get() > Constants.Shooter.AcrossFieldSpinUpTime) {
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
    return ((timer.get() > Constants.Shooter.NoteInAirAcrossFieldTime) && !arm.GetBottomLimitSwitch()); //shots have been fired and arm is back down
  }
}

