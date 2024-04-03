// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurnToRedSource extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final XboxController driverController;
  private final ChassisSpeeds sourceTurnSpeed;

  public TurnToRedSource(SwerveSubsystem m_swerveDrive, XboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    driverController = m_driverController;


    sourceTurnSpeed = new ChassisSpeeds(0,0, 0);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    sourceTurnSpeed.vxMetersPerSecond = 0;
    sourceTurnSpeed.vyMetersPerSecond = 0;
    sourceTurnSpeed.omegaRadiansPerSecond = (swerveDrive.getHeading().getDegrees() - 60)*-Constants.Drivebase.AmpAlignKP; //red 
   
    swerveDrive.drive(sourceTurnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverController.getRawButton(5); //go back to regular driving when you let go of button #5

  }
}
