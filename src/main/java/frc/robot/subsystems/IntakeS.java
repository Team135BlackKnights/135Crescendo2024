package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;



public class IntakeS extends SubsystemBase {
    //declarations of motors/encoders and limit switch
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
    public RelativeEncoder deployIntakeEncoder, primaryIntakeEncoder;
    public SparkAbsoluteEncoder absDeployIntakeEncoder;
    public static ColorSensorV3 colorSensorV3 = new ColorSensorV3(Constants.IntakeConstants.colorSensorPort);
    public static ColorMatch colorMatch = new ColorMatch();
    public static Color detected = new Color();
    public static ColorMatchResult colorMatchResult;
    public Thread sensorUpdateThread;
    int timesRan;
    public IntakeS() {
        timesRan = 0;
        //set note color to color match
        colorMatch.addColorMatch(Constants.IntakeConstants.noteColor);
        colorMatch.addColorMatch(Color.kBlue);
        colorMatch.addColorMatch(Color.kRed);
        colorMatch.addColorMatch(Color.kGray);
        colorMatch.addColorMatch(Color.kWhite);
        //sets intake motors to reversed, sets idleMode to brake
        primaryIntake.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
        deployIntake.setInverted(Constants.IntakeConstants.deployIntakeReversed);

        primaryIntake.setIdleMode(IdleMode.kBrake);
        //creates encoders and makes them work with the gear ratios
        primaryIntakeEncoder = primaryIntake.getEncoder();
        primaryIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);
        primaryIntakeEncoder.setVelocityConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);

        deployIntakeEncoder = deployIntake.getEncoder();
        deployIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.deployIntakeGearRatio);

        absDeployIntakeEncoder = deployIntake.getAbsoluteEncoder(Type.kDutyCycle);
        //sets changes to motor (resource intensive, ONLY CALL ON INITIALIZATION)
        primaryIntake.burnFlash();
        deployIntake.burnFlash();
        sensorUpdateThread = new Thread(() -> {
             detected = colorSensorV3.getColor();
             colorMatchResult = colorMatch.matchClosestColor(detected);
        });
        
    }
    public void updateSensor(){
        sensorUpdateThread.setDaemon(true);
        sensorUpdateThread.run();
    }
    @Override
    public void periodic() {
        timesRan += 1;
        timesRan %= 5;
        if (timesRan == 0){
            updateSensor();
        }
        //sets values to SmartDashboard periodically
        SmartDashboard.putNumber("Deploy Intake", deployIntakeEncoder.getPosition());
        SmartDashboard.putBoolean("Note Loaded?", noteIsLoaded());
        SmartDashboard.putNumber("Red", detected.red);
        SmartDashboard.putNumber("Green", detected.green);
        SmartDashboard.putNumber("Blue", detected.blue);
        
        SmartDashboard.putString("data", colorMatchResult.color.toString());
    }
    
    public void updateSensorColor(){
        sensorUpdateThread.setDaemon(true);
        sensorUpdateThread.run();

    }

    public static boolean noteIsLoaded() {
        //pulls data from color sensor
        
        
        if (colorMatchResult.color == IntakeConstants.noteColor){
            return true;
        }
        else{
            return false;
        }
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
            if (deployIntakeEncoder.getPosition() < 0) {
                power = 0;
            } else if (deployIntakeEncoder.getPosition() < 44.1) {
                power = power * 0.5;
            }
        }
        //second set of conditionals (below) checks to see if the arm is within the hard limits, and stops it if it is
        if (power > 0) {
            if (deployIntakeEncoder.getPosition() > 113) {
                power = 0;
            } else if (deployIntakeEncoder.getPosition() > 79) {
                power = power * 0.5;
            }
        }
        //power value (as a percent) sent to smartDashboard only if intake is called
        SmartDashboard.putNumber("Deploy Intake Percentage", power);

        deployIntake.set(power);
    }
}
