// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Main;
import frc.robot.RobotContainer;
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


    autoAmpSpeeds = new ChassisSpeeds(0,0, 0);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    DriverStation.getAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

//assumes you are blue team

/*if (swerveDrive.getHeading().getDegrees() > -85){
   autoAmpSpeeds.omegaRadiansPerSecond = -2; //turn CCW if heading is less than 85 degees
   }

else if (swerveDrive.getHeading().getDegrees() < -95){ 
  autoAmpSpeeds.omegaRadiansPerSecond = 2; //turn CW if heading is less than 85 degees)
}

else {
  */
  autoAmpSpeeds.omegaRadiansPerSecond = 0; //stop turing
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
