// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class JoystickArmCommand extends Command {
  /** Creates a new JoystickArmCommand. */
  private final ArmSubsystem arm;
  private final XboxController armController;

  public JoystickArmCommand(ArmSubsystem m_arm, XboxController m_armController) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    armController = m_armController;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.ArmJoystickControl(-armController.getRawAxis(3));
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
