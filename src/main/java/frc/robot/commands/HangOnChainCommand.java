// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HangSubsystem;

public class HangOnChainCommand extends Command {
  /** Creates a new HangWithArmCommand. */
  private final HangSubsystem hang;
  private final ArmSubsystem arm;
  private final XboxController operatorController;

  public HangOnChainCommand(HangSubsystem m_hang, ArmSubsystem m_arm, XboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    hang = m_hang;
    arm = m_arm;
    operatorController = m_operatorController;

    addRequirements(hang, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.HangOnChain(arm.GetBottomLimitSwitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.StopHangMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !operatorController.getRawButton(8); //stop command when operator lets go of start button
  }
}
