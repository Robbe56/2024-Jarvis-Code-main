// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RollerButtonCommand extends Command {
  /** Creates a new UnjamCommand. */
  ShooterSubsystem shooter;
  IntakeSubsystem intake;
  ArmSubsystem arm;
  XboxController driveController;
  XboxController operatorController;
  Timer timer;

    public RollerButtonCommand(ShooterSubsystem m_shooter, IntakeSubsystem m_intake, ArmSubsystem m_arm, XboxController m_driver, XboxController m_operator) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveController = m_driver;
    operatorController = m_operator;
    shooter = m_shooter;
    intake = m_intake;
    arm = m_arm;
    timer = new Timer();

    addRequirements(shooter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveController.getYButton() == true){
      intake.IntakeSpitOut();
    }

    if ((driveController.getRawAxis(2) > 0.5 || driveController.getRawAxis(3) > 0.5) 
    && 
    shooter.getNoteSensor() == true && arm.GetBottomLimitSwitch() == false){
      intake.intakeActive();
      shooter.FeedMotorFast();
    }

    if ((
    driveController.getRawAxis(2) < 0.5
    && 
    driveController.getRawAxis(3) < 0.5
    &&
    driveController.getYButton() == false) 
    || 
    shooter.getNoteSensor() == false)
    {
      intake.intakeRest();
    }

    if (operatorController.getLeftBumper() == true){
         timer.start();
      if (timer.get() < Constants.Shooter.BackUpShooterWheelTime){
        shooter.ShooterMotorsBackward();
      }
      else {
        shooter.ShooterIntoSpeakerSpeed();
      }
    }

    if (operatorController.getLeftBumper() == false && operatorController.getRawButton(4) == false){ //not pressing anything
      timer.reset();
      shooter.StopShooter();
    }

    if (operatorController.getRightBumper() == true){
      if (shooter.noteInFeeder.get() == true){ //no note detected by sensor on shooter
      shooter.FeedMotorFast();
      }
      else {
        shooter.StopFeedRoller();
      }

    }
    if (operatorController.getRightBumper() == false && operatorController.getRawButton(4) == false && 
    (driveController.getRawAxis(2) < 0.5 && driveController.getRawAxis(3) < 0.5)){
      shooter.StopFeedRoller();
  
    }
    if (operatorController.getRawButton(4) == true){
      shooter.FeedMotorsBackward();
      shooter.ShooterMotorsBackward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
