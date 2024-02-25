// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

WPI_VictorSPX feedMotor;
WPI_VictorSPX rightShooterMotor;
WPI_VictorSPX leftShooterMotor;

public DigitalInput noteInFeeder;

double armCommandSpeed;

  public ShooterSubsystem() {


feedMotor = new WPI_VictorSPX(Constants.Shooter.feedMotorCANID);
rightShooterMotor = new WPI_VictorSPX(Constants.Shooter.rightShooterMotorID);
leftShooterMotor = new WPI_VictorSPX(Constants.Shooter.leftShooterMotorID);

noteInFeeder = new DigitalInput(Constants.Shooter.noteInFeederSensor);


feedMotor.configFactoryDefault();
rightShooterMotor.configFactoryDefault();
leftShooterMotor.configFactoryDefault();

}
  public void StopFeedRoller(){
    feedMotor.stopMotor();
  }

  public void StopShooter(){
    rightShooterMotor.stopMotor();
    leftShooterMotor.stopMotor();
  }


    /*if (armAtRest.get() && armAtAmp.get() && armCommandSpeed < 0.05 && armCommandSpeed > -0.05) { //not on limit switch and not pressing joystick
      armMotor.set(Constants.Shooter.armHoldSpeed);
    }
    
    if(armCommandSpeed < -0.05 && armAtRest.get() == true){ //pulling back on joystick and not pressing down limit switch
      if (armCommandSpeed < Constants.Shooter.MaxDownSpeed){
        armMotor.set(Constants.Shooter.MaxDownSpeed);
        }
        
      else {armMotor.set(armCommandSpeed);}
    }

    if (armCommandSpeed > 0.05 && armAtAmp.get() == true){ //pushing forward on joystick and not pressing top limit switch
      if (armCommandSpeed > Constants.Shooter.MaxUpSpeed){
        armMotor.set(Constants.Shooter.MaxUpSpeed);
        }
      else {armMotor.set(armCommandSpeed);}
    }

    if ((armCommandSpeed < 0 && armAtRest.get() == false) || (armCommandSpeed > 0 && armAtAmp.get() == false)){
      armMotor.stopMotor();
    }
    */



   public void FeedMotorSlow(){
    feedMotor.set(Constants.Shooter.feedLowSpeed);
   }

    public void FeedMotorFast(){
    feedMotor.set(Constants.Shooter.feedHighSpeed);
   }

   public void FeedMotorsBackward(){
    feedMotor.set(Constants.Shooter.feedBackward);
   }
   
   public void ShooterMotorsBackward(){
    rightShooterMotor.set(-Constants.Shooter.shootHighSpeed);
    leftShooterMotor.set(-Constants.Shooter.shootHighSpeed);
   }

   public void ShooterIntoAmpSpeed(){
    rightShooterMotor.set(Constants.Shooter.shootLowSpeed);
    leftShooterMotor.set(Constants.Shooter.shootLowSpeed);
   }

    public void ShooterIntoSpeakerSpeed(){
    rightShooterMotor.set(Constants.Shooter.shootHighSpeed);
    leftShooterMotor.set(Constants.Shooter.shootHighSpeed);
   }

   public boolean getNoteSensor(){
    return noteInFeeder.get();
   }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Sensor", noteInFeeder.get());

  }
}
