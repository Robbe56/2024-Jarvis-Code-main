// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  WPI_TalonSRX armMotor;

  DigitalInput armAtRest;
  DigitalInput armAtAmp;

  public ArmSubsystem() {
  armMotor = new WPI_TalonSRX(Constants.Shooter.armMotorCANID);

  armAtRest = new DigitalInput(Constants.Shooter.armDownLimitSwitch);
  armAtAmp = new DigitalInput(Constants.Shooter.armUpLimitSwitch);
  
  armMotor.configFactoryDefault();
  armMotor.setSelectedSensorPosition(0);

} 

public void StopArm(){
  armMotor.stopMotor();
}

public void ArmJoystickControl(double armCommandSpeed){

  if ((armCommandSpeed > 0 && armAtAmp.get() == false) || (armCommandSpeed <0 && armAtRest.get() == false)){
   armMotor.set(0);
  }

   else {
     if (armCommandSpeed < -.4){
       armMotor.set(0.4);
     } 
     else{
     armMotor.set(-armCommandSpeed);
     }
   }

   if (armAtRest.get() == false){                      //pushing lower limit switch
    armMotor.setSelectedSensorPosition(0); //reset encoder
  }
}

public void ArmUpCommand(){
  if (armAtAmp.get() == false){ //if pressing top limit switch
    armMotor.stopMotor();
  }
  if (armMotor.getSelectedSensorPosition()/1000 < Constants.Shooter.almostUpValue){ 
    armMotor.set(Constants.Shooter.armUpSpeed);
  }
  if (armMotor.getSelectedSensorPosition()/1000 >= Constants.Shooter.almostUpValue){
    armMotor.set(Constants.Shooter.armUpSpeed*0.2);
  }  
  
 }

 public void ArmDownCommand(){
  if (armAtRest.get() == false){ //if pressing bottom limit switch
    armMotor.stopMotor();
  }
  if (armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostDownValue){ 
    armMotor.set(Constants.Shooter.armDownSpeed);
  }
  if (armMotor.getSelectedSensorPosition()/1000 <= Constants.Shooter.almostDownValue){
    armMotor.set(Constants.Shooter.armDownSpeed*0.15);
  } 
}

public void ArmHoldPosition(){
  armMotor.set(Constants.Shooter.armHoldSpeed);
}

public double GetArmEncoderPosition(){
  return armMotor.getSelectedSensorPosition()/1000;
 }

 public boolean GetTopLimitSwitch(){
  return armAtAmp.get();
 }
  
  public boolean GetBottomLimitSwitch(){
  return armAtRest.get();
 }

 public void ResetArmEncoder(){
  if (armAtRest.get() == false){
    armMotor.setSelectedSensorPosition(0); //reset encoder
  }
 }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition()/1000);
  SmartDashboard.putBoolean("Low Arm Limit Switch", armAtRest.get());
  SmartDashboard.putBoolean("Top Arm Limit Switch", armAtAmp.get());
  SmartDashboard.putNumber("Arm Percentage Setting", armMotor.getMotorOutputPercent());

    
  }
}
