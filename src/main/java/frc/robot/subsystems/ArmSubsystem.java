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

  if ((armCommandSpeed < 0 && armAtAmp.get() == false) || (armCommandSpeed > 0 && armAtRest.get() == false)){
   armMotor.set(0);
  } 
  else if (armCommandSpeed < Constants.Shooter.armUpSpeedMax && armMotor.getSelectedSensorPosition()/1000 < Constants.Shooter.almostUpValue){
    armMotor.set(Constants.Shooter.armUpSpeedMax);
  }
    else if (armCommandSpeed < Constants.Shooter.armUpSpeedMax && armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostUpValue){
    armMotor.set(Constants.Shooter.armUpSpeedMax*Constants.Shooter.UpReductionFactor);
  }
    else if (armCommandSpeed > Constants.Shooter.armDownSpeedMax && armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostDownValue){
    armMotor.set(Constants.Shooter.armDownSpeedMax);
  }
    else if (armCommandSpeed > Constants.Shooter.armDownSpeedMax && armMotor.getSelectedSensorPosition()/1000 < Constants.Shooter.almostDownValue){
    armMotor.set(Constants.Shooter.armDownSpeedMax*Constants.Shooter.DownReductionFactor);
  }
  
    else {
    armMotor.set(armCommandSpeed);
  }

   /* 
  else {
     if (armCommandSpeed < -Constants.Shooter.armDownSpeedMax){ //pulling back on joystick to move arm down at a speed higher than max

      if (armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostDownValue){ //not near the bottom
        armMotor.set(Constants.Shooter.armDownSpeedMax);  //go down at max safe speed
        followerArmMotor.set(Constants.Shooter.armDownSpeedMax); //go down at max safe speed
      }
      else if (armMotor.getSelectedSensorPosition()/1000 < Constants.Shooter.almostDownValue && armAtRest.get() == true){ //near the bottom but not hitting limit switch yet
        armMotor.set(Constants.Shooter.armDownSpeedMax*Constants.Shooter.DownReductionFactor);  //go down at % of max safe speed
        followerArmMotor.set(Constants.Shooter.armDownSpeedMax*Constants.Shooter.DownReductionFactor); //go down at % max safe speed
      }
      
       else if (armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostUpValue){ //not near the top
        armMotor.set(Constants.Shooter.armUpSpeedMax);  //go up at max safe speed
        followerArmMotor.set(Constants.Shooter.armUpSpeedMax); //go up at max safe speed
      }
      else if (armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostDownValue && armAtAmp.get() == true){ //near the top but not hitting limit switch yet
        armMotor.set(Constants.Shooter.armUpSpeedMax*Constants.Shooter.UpReductionFactor);  //go up at % of max safe speed
        followerArmMotor.set(Constants.Shooter.armUpSpeedMax*Constants.Shooter.UpReductionFactor); //go up at % max safe speed
      }
     } 
     else{
     armMotor.set(-armCommandSpeed); //if your not over the max speed just do what the joystick says
     followerArmMotor.set(-armCommandSpeed); //if your not over the max speed just do what the joystick says
     }
   }
   */

   if (armAtRest.get() == false){                      //pushing lower limit switch
    armMotor.setSelectedSensorPosition(0); //reset encoder
  }
}

public void ArmUpCommand(){
  if (armAtAmp.get() == false){ //if pressing top limit switch
    armMotor.stopMotor();
  }
  if (armMotor.getSelectedSensorPosition()/1000 < Constants.Shooter.almostUpValue){ 
    armMotor.set(Constants.Shooter.armUpAutoSpeed);
  }
  /*if (armMotor.getSelectedSensorPosition()/1000 >= Constants.Shooter.almostUpValue){
    armMotor.set(Constants.Shooter.armUpSpeedMax*0.2);
    followerArmMotor.set(Constants.Shooter.armUpSpeedMax*0.2);
  } 
  */ 
  
 }
 public void ArmUpFastCommand(){
  if (armAtAmp.get() == false){ //if pressing top limit switch
    armMotor.stopMotor();

  } else armMotor.set(Constants.Shooter.armUpSpeedMax);

  /*if (armMotor.getSelectedSensorPosition()/1000 >= Constants.Shooter.almostUpValue){
    armMotor.set(Constants.Shooter.armUpSpeedMax*0.2);
    followerArmMotor.set(Constants.Shooter.armUpSpeedMax*0.2);
  } 
  */ 
  
 }

 public void ArmDownCommand(){
  if (armAtRest.get() == false){ //if pressing bottom limit switch
    armMotor.stopMotor();
  }
  if (armMotor.getSelectedSensorPosition()/1000 > Constants.Shooter.almostDownValue){ 
    armMotor.set(Constants.Shooter.armDownSpeedMax);
  }
  if (armMotor.getSelectedSensorPosition()/1000 <= Constants.Shooter.almostDownValue){
    armMotor.set(Constants.Shooter.armDownSpeedMax*0.15);
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
