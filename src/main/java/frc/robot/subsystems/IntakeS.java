package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.DriveSimConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.autoCommands.AutonIntake;




public class IntakeS extends SubsystemBase {
    //declarations of motors/encoders and limit switch
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
    public static RelativeEncoder deployIntakeEncoder;
    public RelativeEncoder primaryIntakeEncoder;
    public static DutyCycleEncoder absDeployIntakeEncoder;
    public static ColorSensorV3 colorSensorV3 = new ColorSensorV3(Constants.IntakeConstants.colorSensorPort);
    public static ColorMatch colorMatch = new ColorMatch();
    public static Color detected = Color.kBlack;
    public static ColorMatchResult colorMatchResult;
    public static Thread sensorThread;
    public static int timesRan;
    public PIDController anglePidController = new PIDController(0.06, 0, 0);
    private static double kP,kI,kD;

    public static PIDController autoIntakeController = new PIDController(0.00135, 0.00135, 0.00001); //sadly cannot be system Id'd

    public IntakeS() {
        timesRan = 0;

        colorMatch.addColorMatch(Constants.IntakeConstants.noteColor);
        colorMatch.addColorMatch(Color.kBlue);
        colorMatch.addColorMatch(Color.kRed);
        colorMatch.addColorMatch(Color.kGray);
        colorMatch.addColorMatch(Color.kWhite);

        //sets intake motors to reversed, sets idleMode to brake
        primaryIntake.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
        deployIntake.setInverted(Constants.IntakeConstants.deployIntakeReversed);

        primaryIntake.setIdleMode(IdleMode.kBrake);
        deployIntake.setIdleMode(IdleMode.kBrake);

        //creates encoders and makes them work with the gear ratios
        primaryIntakeEncoder = primaryIntake.getEncoder();
        primaryIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);
        primaryIntakeEncoder.setVelocityConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);

        deployIntakeEncoder = deployIntake.getEncoder();

        absDeployIntakeEncoder = new DutyCycleEncoder(Constants.IntakeConstants.intakeAbsEncoderID);
        //sets changes to motor (resource intensive, ONLY CALL ON INITIALIZATION)
        primaryIntake.burnFlash();
        deployIntake.burnFlash();
        kP = Constants.IntakeConstants.PIDConstants.P;
        kI = Constants.IntakeConstants.PIDConstants.I;
        kD = Constants.IntakeConstants.PIDConstants.D;
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        //Color sensor thread

    }
    @Override
    public void periodic() {
        
        //sets values to SmartDashboard periodically
      //  SmartDashboard.putNumber("Deploy Intake", deployIntakeEncoder.getPosition());
        SmartDashboard.putNumber("Deploy Intake Abs", getIntakePosition());
        SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
      //  SmartDashboard.putBoolean("Intake Within Bounds", intakeWithinBounds());
        SmartDashboard.putNumber("Intake Offset", IntakeConstants.intakeOffset);
        double p = SmartDashboard.getNumber("P Gain", Constants.IntakeConstants.PIDConstants.P);
        double i = SmartDashboard.getNumber("I Gain", Constants.IntakeConstants.PIDConstants.I);
        double d = SmartDashboard.getNumber("D Gain", Constants.IntakeConstants.PIDConstants.D);
        if ((p != kP)) { 
            autoIntakeController.setP(p);
             kP = p; 
        }
        if ((i != kI)) {
            autoIntakeController.setI(i); kI = i; 
        }
        if ((d != kD)) {
            autoIntakeController.setD(d); kD = d;
        }
    }

    public static double getIntakePosition() {
        return absDeployIntakeEncoder.getAbsolutePosition()*Constants.IntakeConstants.absIntakeEncoderConversionFactor - Constants.IntakeConstants.absIntakeEncoderOffset;
    }

    public double getIntakeAngle() {
        return getIntakePosition()-IntakeConstants.intakeOffset;
    }

    public boolean intakeWithinBounds() {
        if (Robot.isSimulation()) return true;
        return getIntakeAngle() > CameraS.getDesiredShooterLowerBound() && getIntakeAngle() < CameraS.getDesiredShooterUpperBound() || (getIntakeAngle() > 42 && CameraS.getDesiredShooterAngle() > 42);
    }
    

    public static boolean noteIsLoaded() {
        if (colorMatchResult.color == IntakeConstants.noteColor){
            return true;
        }
        else{
            return false;
        }

    }
    public Translation2d getClosestNote(){
        //Obnoxiously high distance to be overrode
        Translation2d closestTrans = new Translation2d();
        double closestPoseDistance = 9999;
        for (var pose : DriveSimConstants.fieldNotePoses){
            if (pose.getDistance(SwerveS.getPose().getTranslation())< closestPoseDistance){
                closestTrans = pose;
                closestPoseDistance = pose.getDistance(SwerveS.getPose().getTranslation());
            }
        }
        return closestTrans;

    }

    public void setPrimaryIntake(double power) {
        // sets the primary intake, comment below is a deadband check
        //power = power <= 0.1 ? 0.1 : power;
        primaryIntake.set(power);
    }

    public void deployIntake(double power) {
        //release the ̶h̶o̶r̶d̶e̶ intake 
        //first set of conditionals checks to see whether the arm is within the soft limits, and slows it down if it isnt
        if (power < 0) {
            if (getIntakePosition() < IntakeConstants.deployIntakeInnerBound) {
                power = 0;
            } else if (getIntakePosition() < IntakeConstants.deployIntakeOuterBound*0.33) {
                power = power * 0.5;
            }
        }
        //second set of conditionals (below) checks to see if the arm is within the hard limits, and stops it if it is
        if (power > 0) {
            if (getIntakePosition() > IntakeConstants.deployIntakeOuterBound) {
                power = 0;
            } else if (getIntakePosition() > IntakeConstants.deployIntakeOuterBound*0.67) {
                power = power * 0.5;
            }
        }
        //power value (as a percent) sent to smartDashboard only if intake is called
       // SmartDashboard.putNumber("Deploy Intake Percentage", power);

        deployIntake.set(power);
    }
    public void pullBackNote(){
        new Thread(() -> {
            Timer timer = new Timer();
            timer.start();
            while (timer.get() < .15) {
                setPrimaryIntake(-0.5);
            }
            timer.reset();
            while (timer.get() < .15){
                setPrimaryIntake(0.2);
            }
            setPrimaryIntake(0);
            AutonIntake.allClear = true;
        }).start();
    }
    }