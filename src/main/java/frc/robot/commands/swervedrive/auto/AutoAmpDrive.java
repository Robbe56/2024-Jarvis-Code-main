// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;


import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAmpDrive extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final XboxController driverController;
  private final Timer timer;
  private final ChassisSpeeds autoAmpSpeeds;

 
  public AutoAmpDrive(SwerveSubsystem m_swerveDrive, XboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    driverController = m_driverController;
    timer = new Timer();


    autoAmpSpeeds = new ChassisSpeeds(0,0,0);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /*
  Optional<Alliance> color = DriverStation.getAlliance();
  if (color.isPresent()) {
    if (color.get() == Alliance.Red) {
        SmartDashboard.putString("Alliance Color", "Red");
    }
    if (color.get() == Alliance.Blue) {
      SmartDashboard.putString("Alliance Color", "Blue");
    }

    else {
      SmartDashboard.putString("Alliance Color", "No Alliance Color Yet");
    }
  }
  */
 

//now set up for red
//if red alliance, autoAmpSpeeds.omegaRadiansPerSecond = (swerveDrive.getHeading().getDegrees() - 90)*-Constants.Drivebase.AmpAlignKP;
  autoAmpSpeeds.omegaRadiansPerSecond = (swerveDrive.getHeading().getDegrees() - 90)*-Constants.Drivebase.AmpAlignKP; //red 
  //autoAmpSpeeds.omegaRadiansPerSecond = (90 + swerveDrive.getHeading().getDegrees())*-Constants.Drivebase.AmpAlignKP; //blue
  autoAmpSpeeds.vxMetersPerSecond = Constants.Drivebase.AmpForwardSpeed;
  autoAmpSpeeds.vyMetersPerSecond = Constants.Drivebase.AmpKP*swerveDrive.TrackSpeaker(); //multiply Limelight value by P factor

       
swerveDrive.drive(autoAmpSpeeds);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driverController.getRawButton(3); //go back to regular driving when you let go of button #3

  }
}
