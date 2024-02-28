// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    public static HashMap<String, Command> eventMap = new HashMap<>();
  }

  public static class IntakeConstants {

    public static I2C.Port colorSensorPort = I2C.Port.kOnboard;
    public static Color noteColor = new Color(0.55, 0.36, .08);

    public static int
      primaryIntakeID = 20,
      deployIntakeID = 21,
      intakeLimitSwitchID = 9;

    public static double
      primaryIntakeGearRatio = 1/4.5,
      deployIntakeGearRatio = 0;

    public static boolean
      primaryIntakeReversed = true,
      deployIntakeReversed = false;
  
  }

  public static class OutakeConstants {
    public static int
      topFlywheel = 30,
      bottomFlywheel = 31;

    public static double
      flywheelMaxRPM = 6600,
      flywheelGearRatio = 1.5,
      idealPercentTop = .0291,
      idealPercentBottom = .241;

    public static boolean
      topFlywheelReversed = true,
      bottomFlywheelReversed = true;
    
  }

  public static class SwerveConstants {
    public static double
      kWheelDiameter = Units.inchesToMeters(4), 
      kDriveMotorGearRatio = 1/8.14, 
      kTurningMotorGearRatio = (7/150),
      kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameter,
      kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60,
      kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI,
      kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60,
      kDeadband = 0.1,
      kAutoDeadband = 0.01,

      kTurningP = 0.4;
  }

  public static class DriveConstants {
    public static double
      kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
      kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels

      kDriveBaseRadius = Units.inchesToMeters(Math.sqrt(kChassisLength*kChassisLength + kChassisWidth*kChassisWidth)/2),
      // Distance from center of robot to the fartshest module

      kMaxSpeedMetersPerSecond = Units.feetToMeters(12.0),
      kMaxTurningSpeedRadPerSec = 4.414667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
      kTeleDriveMaxAcceleration = Units.feetToMeters(5.66), //guess
      kTeleTurningMaxAcceleration = 5, //guess
      
      // To find these set them to zero, then turn the robot on and manually set the wheels straight.
      // The encoder values being read are then your new Offset values
      kFrontLeftAbsEncoderOffsetRad = 2*Math.PI - 1.993532,
      kFrontRightAbsEncoderOffsetRad = 1.647362,
      kBackLeftAbsEncoderOffsetRad = 2*Math.PI - 0.978094,
      kBackRightAbsEncoderOffsetRad = 2*Math.PI - 1.332261; 
    
    // Declare the position of each module
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kChassisLength / 2, kChassisWidth / 2),
      new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
      new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
      new Translation2d(-kChassisLength / 2, -kChassisWidth / 2));

    public static int
      kFrontLeftDrivePort = 10, //10
      kFrontLeftTurningPort = 11, //20
      kFrontLeftAbsEncoderPort = 0, //1

      kFrontRightDrivePort = 12, //11
      kFrontRightTurningPort = 13, //21
      kFrontRightAbsEncoderPort = 1, //2

      kBackLeftDrivePort = 16, //13
      kBackLeftTurningPort = 17, //23
      kBackLeftAbsEncoderPort = 2, //3

      kBackRightDrivePort = 14, //14
      kBackRightTurningPort = 15, //24
      kBackRightAbsEncoderPort = 3; //4

    public static boolean
      kFrontLeftDriveReversed = false,
      kFrontLeftTurningReversed = true,
      kFrontLeftAbsEncoderReversed = false,

      kFrontRightDriveReversed = false,
      kFrontRightTurningReversed = true,
      kFrontRightAbsEncoderReversed = false,

      kBackLeftDriveReversed = false,
      kBackLeftTurningReversed = true,
      kBackLeftAbsEncoderReversed = false,

      kBackRightDriveReversed = false,
      kBackRightTurningReversed = true,
      kBackRigthAbsEncoderReversed = false;

  }

  public static class LEDConstants{
    //basically anything pertaining to the LEDs go in here
    public static int 
    ledPort = 9, 
    ledBufferLength = 90, // amount of LEDs in the light strip
    noteH = 19, //note HSV value
    noteS = 255,
    noteV = 100,
    redH = 0, //red HSV valuess
    redS = 255,
    redV = 100,
    blueH = 120, //blue HSV values
    blueS = 255,
    blueV = 100,
    greenH = 50,//green hsv values
    greenS = 255,
    greenV = 100;
     
    public static double sinePeriod = 16; 
    //Basically controls how different the waves are from one another when the setColorWave function is called. Due to how it is calculated , this value CANNOT be zero (divide by zero error). 
    //Try to set this value to a multiple of however many LEDs we have (so like if we have 63 LEDs on the robot set the sine to 9)
  }
  public static class HangConstants{
    public static double
      hangLowerSoftStop = 12,
      hangUpperSoftStop = 87; //Note: for some reason left and right encoders output different values, MAYBE change them to have left and right max?

    public static int
      leftHangID = 41,
      rightHangID = 40;

    public static boolean
      leftHangReversed = true,
      rightHangReversed = false;
  }
  public static class LimelightConstants{
    public static double limeLightAngleOffsetDegrees = 70,
    limelightLensHeightoffFloorInches = 18,
    targetHeightoffFloorInches = 84.6841;
  }
  
}
