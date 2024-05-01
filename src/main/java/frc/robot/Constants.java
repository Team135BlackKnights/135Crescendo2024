// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
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
  public static final Mode currentMode = Mode.SIM;
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static class IntakeConstants {

    public static I2C.Port colorSensorPort = I2C.Port.kOnboard;
    public static Color noteColor = new Color(0.55, 0.36, .08);

    public static int
      primaryIntakeID = 20,
      deployIntakeID = 21,
      intakeAbsEncoderID = 1,
      intakeLimitSwitchID = 9,
      intakeOffset = 43;

    public static double
      absIntakeEncoderOffset = 59.870652,
      absIntakeEncoderConversionFactor = 360,
      primaryIntakeGearRatio = 1/4.5,
      deployIntakeInnerBound = 0,
      deployIntakeOuterBound = 90;

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
      shooterHeight = Units.inchesToMeters(15),
      flywheelMaxRPM = 7100,
      flywheelGearRatio = 1.5,
      idealPercentTop = .34,
      idealPercentBottom = .31,
      kP = 0.0021258,
      kSVolts = -0.089838,
      kVVoltSecondsPerRotation= 0.0015425*.928,
      kAVoltSecondsSquaredPerRotation = 0.00039717*1;

    public static boolean
      topFlywheelReversed = false,
      bottomFlywheelReversed = false;
    
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
      kAutoDeadband = 0.01;

      
  }

  public static class DriveConstants {
    public static double
      kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
      kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels

      kDriveBaseRadius = Units.inchesToMeters(Math.sqrt(kChassisLength*kChassisLength + kChassisWidth*kChassisWidth)/2),
      // Distance from center of robot to the farthest module

      kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1),
      kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
      kTeleDriveMaxAcceleration = Units.feetToMeters(12), //guess
      kTeleTurningMaxAcceleration = 2 * Math.PI, //guess
      
      // To find these set them to zero, then turn the robot on and manually set the wheels straight.
      // The encoder values being read are then your new Offset values
      kFrontLeftAbsEncoderOffsetRad = 0.562867,
      kFrontRightAbsEncoderOffsetRad = 0.548137,
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
    public static double 
      kFrontRightP = 2.4646, //2.4646 maybe
      kFrontRightSVolts = -0.040248,
      kFrontRightVVoltSecondsPerRotation = 2.9041,
      kFrontRightAVoltSecondsSquaredPerRotation = 1.52,
      
      kFrontLeftP = 2.5896, //2.5896 //4.1054
      kFrontLeftSVolts =-0.22934,
      kFrontLeftVVoltSecondsPerRotation = 2.8559,
      kFrontLeftAVoltSecondsSquaredPerRotation = 1.7338,

      kBackRightP = 2.0873,//maybe 2.0873 or 1.4862 //4.1688
      kBackRightSVolts =0.070421,
      kBackRightVVoltSecondsPerRotation = 2.8607,
      kBackRightAVoltSecondsSquaredPerRotation =  1.1811,

      kBackLeftP = 2.3375,//maybe 2.3375 or 1.5638 //3.9698
      kBackLeftSVolts = 0.01842,
      kBackLeftVVoltSecondsPerRotation = 2.7005,
      kBackLeftAVoltSecondsSquaredPerRotation = 1.4511;
      ;
  }

  public static class LEDConstants{
    //basically anything pertaining to the LEDs go in here
    public static int 
    //ledPort
    ledPort = 9, 
    // amount of LEDs in the light strip
    ledBufferLength = 90,

    sineWaveUpdateCycles = 1,
    overrideLEDPatternTime = 4;

    //all arrays below use the H,S,V format
    public static int[] 
    noteHSV = new int[]{12, 255, 100},
    redHSV = new int[]{0,255,100},
    blueHSV = new int[]{120,255,100},
    greenHSV = new int[]{50,255,100},
    pinkHSV = new int[]{147,255,255},
    goldHSV = new int[]{23,255,100},
    disabledHSV = new int[]{0,0,0};
    public static int sinePeriod = 32;
    
    public static int[] ledStates = new int[LEDConstants.sinePeriod];
   
    //Basically controls how different the waves are from one another when the setColorWave function is called. Due to how it is calculated , this value CANNOT be zero (divide by zero error). 
    //Try to set this value to a multiple of however many LEDs we have (so like if we have 63 LEDs on the robot set the sine to 9)
    
    
    //goes half as fast in idle
  }
  public static class HangConstants{
    public static double
      upperHookHeight = 63,
      hangLowerSoftStop = 5,
      hangUpperSoftStop = 101; //Note: for some reason left and right encoders output different values, MAYBE change them to have left and right max?
      //left side is left, right side is right

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
      speakerLowerLipHeight = Units.inchesToMeters(78.13),
      speakerUpperLipHeight = Units.inchesToMeters(80.9),
      noteHeight = Units.inchesToMeters(2.5),
      speakerOpeningDepth = Units.inchesToMeters(18.11);

    public static Translation2d
      blueSpeaker = new Translation2d(0, 5.56),
      redSpeaker = new Translation2d(16.59128, 5.56);

  }
  public static class DataLog{
    public static Map<Integer, String> manCanIdsToNames(){
      HashMap<Integer,String> map = new HashMap<>();
      map.put(Constants.OutakeConstants.topFlywheel,"topFlywheel");
      return map;
    }
    public static int testRunSeconds = 10;
    public static double[][] variableAngleLog = new double[2][20];
    public static double variableAngleDistance = 0;
    public static double angleOutputDegrees = 0;
  }
  
}