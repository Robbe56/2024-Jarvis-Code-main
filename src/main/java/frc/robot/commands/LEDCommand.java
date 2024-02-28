// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterSubsystem;

public class LEDCommand extends Command {
  /** Creates a new LEDCommand. */
  private final LEDs leds;
  private final ShooterSubsystem shooter;

  public LEDCommand(LEDs m_leds, ShooterSubsystem m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
  leds = m_leds;
  shooter = m_shooter;

  addRequirements(leds, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.noteLight(shooter.getNoteSensor());
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
