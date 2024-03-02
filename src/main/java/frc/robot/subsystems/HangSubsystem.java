// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
   VictorSP hangMotor;
  
  public HangSubsystem() {
    hangMotor = new VictorSP(Constants.Hang.HangMotorPWMID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void HangOnChain(boolean bottomLimitSwitch){
    if (bottomLimitSwitch == false){ //if bottom limit switch is pressed
      hangMotor.stopMotor();
      }
    if (bottomLimitSwitch == true){ //if bottom limit switch is not being pressed 
      hangMotor.set(Constants.Hang.armHangSpeed);
    }
  }

  public void ResetHangerMotor(){
    hangMotor.set(Constants.Hang.hangerUnwindSpeed);
  }

  public void StopHangMotor(){
    hangMotor.stopMotor();
  }
}
