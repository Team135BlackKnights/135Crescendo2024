// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

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

    public static int colorSensorPort = 0;

    public static int
      primaryIntakeID = 20,
      deployIntakeID = 21,
      intakeAbsEncoderID = 1,
      intakeLimitSwitchID = 9;

    public static double
      absIntakeEncoderOffset = 20.356957,
      absIntakeEncoderConversionFactor = 360,
      primaryIntakeGearRatio = 1/4.5,
      deployIntakeInnerBound = 0,
      deployIntakeOuterBound = 91;

    public static boolean
      primaryIntakeReversed = true,
      deployIntakeReversed = false;
  
  }

  public static class OutakeConstants {
    public static int
      topFlywheel = 30,
      bottomFlywheel = 31;

    public static double
      limelightToShooter = Units.inchesToMeters(-3),
      flywheelMaxRPM = 6600,
      flywheelGearRatio = 1.5,
      idealPercentTop = .034,
      idealPercentBottom = .31;

    public static boolean
      topFlywheelReversed = true,
      bottomFlywheelReversed = true;
    
  }

  public static class SwerveConstants {
    public static double
      kWheelDiameter = Units.inchesToMeters(3.873), 
      kDriveMotorGearRatio = 1/6.75, 
      kTurningMotorGearRatio = (7/150),
      kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameter,
      kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60,
      kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI,
      kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60,
      kDeadband = 0.1,
      kAutoDeadband = 0.01,

      kTurningP = 0.5;
  }

  public static class DriveConstants {
    public static double
      kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
      kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels

      kDriveBaseRadius = Units.inchesToMeters(Math.sqrt(kChassisLength*kChassisLength + kChassisWidth*kChassisWidth)/2),
      // Distance from center of robot to the fartshest module

      kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1),
      kMaxTurningSpeedRadPerSec = 4.414667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
      kTeleDriveMaxAcceleration = Units.feetToMeters(5.66), //guess
      kTeleTurningMaxAcceleration = 5, //guess
      
      // To find these set them to zero, then turn the robot on and manually set the wheels straight.
      // The encoder values being read are then your new Offset values
      kFrontLeftAbsEncoderOffsetRad = 0.562867,
      kFrontRightAbsEncoderOffsetRad = 2.610261,
      kBackLeftAbsEncoderOffsetRad = 2*Math.PI - 2.891372,
      kBackRightAbsEncoderOffsetRad = 2*Math.PI - 0.116861; 
    
    // Declare the position of each module
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kChassisLength / 2, kChassisWidth / 2),
      new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
      new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
      new Translation2d(-kChassisLength / 2, -kChassisWidth / 2));

    public static int
      kFrontLeftDrivePort = 16, //10
      kFrontLeftTurningPort = 17, //20
      kFrontLeftAbsEncoderPort = 2, //1

      kFrontRightDrivePort = 10, //11
      kFrontRightTurningPort = 11, //21
      kFrontRightAbsEncoderPort = 0, //2

      kBackLeftDrivePort = 14, //13
      kBackLeftTurningPort = 15, //23
      kBackLeftAbsEncoderPort = 3, //3

      kBackRightDrivePort = 12, //14
      kBackRightTurningPort = 13, //24
      kBackRightAbsEncoderPort = 1; //4

    public static boolean
      kFrontLeftDriveReversed = true,
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
    //ledPort
    ledPort = 9, 
    // amount of LEDs in the light strip
    ledBufferLength = 90,
    sineWaveUpdateCycles = 3;
    //all arrays below use the H,S,V format
    public static int[] 
    noteHSV = new int[]{12, 255, 100},
    redHSV = new int[]{0,255,100},
    blueHSV = new int[]{120,255,100},
    greenHSV = new int[]{50,255,100},
    goldHSV = new int[]{23,255,100};
  
  
    

    public static int sinePeriod = 32;
    public static int[] ledStates = new int[LEDConstants.sinePeriod];
   
    //Basically controls how different the waves are from one another when the setColorWave function is called. Due to how it is calculated , this value CANNOT be zero (divide by zero error). 
    //Try to set this value to a multiple of however many LEDs we have (so like if we have 63 LEDs on the robot set the sine to 9)
    
    
    //goes half as fast in idle
  }
  public static class HangConstants{
    public static double
      hangLowerSoftStop = 5,
      hangUpperSoftStop = 101; //Note: for some reason left and right encoders output different values, MAYBE change them to have left and right max?

    public static int
      leftHangID = 41,
      rightHangID = 40;

    public static boolean
      leftHangReversed = true,
      rightHangReversed = false;
  }
  public static class LimelightConstants{
    public static double limeLightAngleOffsetDegrees = 15,
    limelightLensHeightoffFloorInches = 22.5;
  }

  public static class FieldConstants {
    public static double
      targetHeightoffFloorInches = 57,
      speakerLowerLipHeight = Units.inchesToMeters(78),
      speakerUpperLipHeight = Units.inchesToMeters(83),
      noteHeight = Units.inchesToMeters(2.5),
      speakerOpeningDepth = Units.inchesToMeters(17);

  }
  
}