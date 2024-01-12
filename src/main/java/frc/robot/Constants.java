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

      kTurningP = 0.5;
  }

  public static class DriveConstants {
    public static double
      kChassisWidth = Units.inchesToMeters(23), // Distance between Left and Right wheels
      kChassisLength = Units.inchesToMeters(23), // Distance betwwen Front and Back wheels

      kDriveBaseRadius = Units.inchesToMeters(Math.sqrt(kChassisLength*kChassisLength + kChassisWidth*kChassisWidth)/2),
      // Distance from center of robot to the fartshest module

      kMaxSpeedMetersPerSecond = Units.feetToMeters(12.0),
      kMaxTurningSpeedRadPerSec = 4.414667 * 2 * Math.PI, //Units.feetToMeters(12) / 360 *2 *Math.PI
      kTeleDriveMaxAcceleration = Units.feetToMeters(5.66), //guess
      kTeleTurningMaxAcceleration = 1, //guess
      
      // To find these set them to zero, then turn the robot on and manually set the wheels straight.
      // The encoder values being read are then your new Offset values
      kFrontLeftAbsEncoderOffsetRad = 3.053225,
      kFrontRightAbsEncoderOffsetRad = 1.51979,
      kBackLeftAbsEncoderOffsetRad = (2*Math.PI) - 0.336506,
      kBackRightAbsEncoderOffsetRad = 1.094702; 
    
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
      kBackRightAbsEncoderPort = 3, //4

      kTestMotorPort = 30; //test!

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
}
