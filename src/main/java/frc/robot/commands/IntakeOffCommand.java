// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeOffCommand extends Command {
  /** Creates a new IntakeOffCommand. */
private final IntakeSubsystem intake;
private final ShooterSubsystem shooter;
private final XboxController controller;

  public IntakeOffCommand(IntakeSubsystem m_intake, ShooterSubsystem m_shooter, XboxController m_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = m_intake;
    shooter = m_shooter;
    controller = m_controller;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeRest();
    
    if (controller.getLeftBumper() == false){
    shooter.StopFeedRoller();
    }
    if (controller.getRightBumper() == true){
      shooter.FeedMotorFast();
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
