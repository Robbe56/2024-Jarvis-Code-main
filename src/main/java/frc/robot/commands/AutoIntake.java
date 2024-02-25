// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  private final Timer timer;

  public AutoIntake(IntakeSubsystem m_intake, ShooterSubsystem m_shooter, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = m_intake;
    shooter = m_shooter;
    arm = m_arm;
    timer = new Timer();

    addRequirements(intake, shooter, arm);
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
    if (arm.GetBottomLimitSwitch() == false){
      intake.intakeRest();
      shooter.StopFeedRoller();
      arm.ArmDownCommand();
    }
    else{
      intake.intakeActive();
      shooter.FeedMotorFast();
      arm.StopArm();
      arm.ResetArmEncoder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeRest();
    shooter.StopFeedRoller();
    arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Intake.IntakeWatchdog;
  }
}
