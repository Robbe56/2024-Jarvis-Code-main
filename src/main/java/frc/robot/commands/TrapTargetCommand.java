// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrapTargetCommand extends Command {
  /** Creates a new MoveArmToSpeakerShot. */
  ArmSubsystem arm;
  ShooterSubsystem shooter;
  XboxController operatorController;
  Timer timer;

  public TrapTargetCommand(ArmSubsystem m_arm, ShooterSubsystem m_shooter, XboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    operatorController = m_operatorController;
    shooter = m_shooter;
    timer = new Timer();
    addRequirements(arm);
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

    //if (arm.GetArmEncoderPosition() < Constants.Shooter.armEquationSlope*swerveDrive.TrackSpeakerHeight() + Constants.Shooter.armEquationIntercept){
    if (arm.GetArmEncoderPosition() < Constants.Shooter.TrapEncoderSetting){
      arm.ArmUpCommand();
    }
    else {arm.ArmHoldPosition();}

    if (timer.get() < Constants.Shooter.TrapTime){
    shooter.ShootIntoTrapSpeed();
    shooter.StopFeedRoller();
  }

  if (timer.get() > Constants.Shooter.TrapTime){
    shooter.ShootIntoTrapSpeed();
    shooter.FeedMotorFast();
  }

 
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.StopArm();
    timer.stop();
    shooter.StopFeedRoller();
    shooter.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !operatorController.getRawButton(9);
  }
}
