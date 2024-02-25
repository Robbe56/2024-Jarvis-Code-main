// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeOnCommand extends Command {
  /** Creates a new IntakeControl. */
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;

  public IntakeOnCommand(IntakeSubsystem m_intake, ShooterSubsystem m_shooter) {
    intake = m_intake;
    shooter = m_shooter;
  
    addRequirements(intake, shooter);
   // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  if(shooter.noteInFeeder.get() == false){ //if Feeder Sensor sees a note
    intake.intakeRest();
    }

  if(shooter.noteInFeeder.get() == true){ //if Feeder Sensor does not see a note
    intake.intakeActive();
    shooter.FeedMotorFast();
    }
   
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
