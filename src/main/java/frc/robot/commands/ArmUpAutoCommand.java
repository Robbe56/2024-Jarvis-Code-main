// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class ArmUpAutoCommand extends Command {
  /** Creates a new ArmUpAutoCommand. */
  private final ArmSubsystem arm;

  public ArmUpAutoCommand(ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.ArmUpCommand();
  }  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !arm.GetTopLimitSwitch(); //returns true when top limit switch is pressed (limit switch goes from true to false with "!")
}
}
