// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveSlow extends Command {
  /** Creates a new HangWithArmCommand. */
  private final SwerveSubsystem swerveDrive;
  private final XboxController driverController;

  public DriveSlow(SwerveSubsystem m_swerveDrive, XboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
  swerveDrive = m_swerveDrive;
  driverController = m_driverController;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  swerveDrive.driveCommand(() ->Constants.Drivebase.SlowDown*driverController.getLeftY(), () ->Constants.Drivebase.SlowDown*driverController.getLeftX(), () ->Constants.Drivebase.SlowDown*-driverController.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverController.getRawButton(5); //stop command when operator lets go of start button
  }
}
