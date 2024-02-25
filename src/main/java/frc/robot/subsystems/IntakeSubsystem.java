// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
WPI_VictorSPX topMotor;
WPI_VictorSPX bottomMotor;


  public IntakeSubsystem() {
  topMotor = new WPI_VictorSPX(Constants.Intake.topMotorCANID);
  bottomMotor = new WPI_VictorSPX(Constants.Intake.bottomMotorCANID);
  }

  public void intakeRest()
  {
  topMotor.stopMotor();
  bottomMotor.stopMotor();
  }

public void intakeActive()
  {
  topMotor.set(Constants.Intake.topMotorIntakeSpeed);
  bottomMotor.set(Constants.Intake.bottomMotorIntakeSpeed);
  }

  public void IntakeSpitOut()
  {
    topMotor.set(-Constants.Intake.topMotorIntakeSpeed);
    bottomMotor.set(-Constants.Intake.bottomMotorIntakeSpeed);
  }

public void intakeSpitOut(){
  topMotor.set(Constants.Intake.SpitOutSpeed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
