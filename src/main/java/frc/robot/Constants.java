// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.02, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.04, 0, 0.01);

    public static final double MAX_ACCELERATION = .1;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class Intake{
    public static final int topMotorCANID = 22;
    public static final int bottomMotorCANID = 21;
    public static final double topMotorIntakeSpeed = 1;
    public static final double bottomMotorIntakeSpeed = 1;
    public static final int SpinButton = 1;
    public static final double SpitOutSpeed = -1;
    public static final double IntakeWatchdog = 5;//intake will run for this long then command will stop
    
  }

  public static final class Shooter{
    public static final int armMotorCANID = 23;
    public static final int followerMotorPWMID = 1;
    public static final int feedMotorCANID = 26;
    public static final int leftShooterMotorID = 25;
    public static final int rightShooterMotorID = 24;

    public static final int armDownLimitSwitch = 8;
    public static final int armUpLimitSwitch = 9;
    public static final int noteInFeederSensor = 0;

    //Arm Encoder Values
    public static final int almostUpValue = 240;
    public static final int almostDownValue = 90;
    public static final int aimedAtSpeaker = 94; // was 94 to shoot from subwoofer, 144 to shoot from safe zone
    public static final int aimedFromSafe = 144; //Shot from safe zone
    
    public static final double ampFeedSpeed = 0.8;
    public static final double ampShooterSpeed = -0.6;
    public static final double waitTimeForScore = 1;

    public static final double armUpSpeedMax = -0.5;
    public static final double armDownSpeedMax = 0.3;
    public static final double DownReductionFactor = 0.15; //15% of speed when going down
    public static final double UpReductionFactor = 0.20; //20% of speed when going up
    public static final double armHangSpeed = 0.9;
    
    public static final double feedLowSpeed = -0.5;
    public static final double feedHighSpeed = -1;
    public static final double shootLowSpeed = -0.4;
    public static final double shootHighSpeed = -0.8;

    public static final double TimeToRunShooterIntoAmp = 1; //run motors for this many seconds
    public static final double feedBackward = .6;
    public static final double BackUpShooterWheelTime = 0; //run motors backward to get note out of shooter wheels before we run them
    
    
    //auto mode values
    public static final double ShooterSpinUpTime = 1.5;
    public static final double NoteInAirTime = 2;
    
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.05;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }

}
