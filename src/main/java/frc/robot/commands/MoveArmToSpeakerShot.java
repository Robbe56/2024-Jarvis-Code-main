// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToSpeakerShot extends Command {
  /** Creates a new MoveArmToSpeakerShot. */
  ArmSubsystem arm;
  XboxController operatorController;

  public MoveArmToSpeakerShot(ArmSubsystem m_arm, XboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    operatorController = m_operatorController;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.GetArmEncoderPosition() < Constants.Shooter.aimedAtSpeaker){
      arm.ArmUpCommand();
    }
    else {arm.ArmHoldPosition();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !operatorController.getRawButton(3);
  }
}
