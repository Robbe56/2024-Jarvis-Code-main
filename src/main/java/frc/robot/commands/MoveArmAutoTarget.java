// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveArmAutoTarget extends Command {
  /** Creates a new MoveArmToSpeakerShot. */
  ArmSubsystem arm;
  SwerveSubsystem swerveDrive;
  XboxController operatorController;

  public MoveArmAutoTarget(ArmSubsystem m_arm, SwerveSubsystem m_swerveDrive, XboxController m_operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    operatorController = m_operatorController;
    swerveDrive = m_swerveDrive;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (arm.GetArmEncoderPosition() < Constants.Shooter.armEquationSlope*swerveDrive.TrackSpeakerHeight() + Constants.Shooter.armEquationIntercept){
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
    return !operatorController.getRawButton(1);
  }
}
