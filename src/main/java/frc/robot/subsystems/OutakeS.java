package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutakeS extends SubsystemBase {
    //motor declarations
   
    //public PIDController shooterPID = new PIDController(0.0000005, 0, 0);


    public CANSparkMax topFlywheel = new CANSparkMax(Constants.OutakeConstants.topFlywheel, MotorType.kBrushless);
    public CANSparkMax bottomFlywheel = new CANSparkMax(Constants.OutakeConstants.bottomFlywheel, MotorType.kBrushless);
    public static SparkPIDController topPIDController;
    public static SparkPIDController bottomPIDController;
    
    //encoder declarations
    public static RelativeEncoder 
    topFlywheelEncoder,
    bottomFlywheelEncoder;

    //pid tuning
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc, minVel, allowedErr;

    public OutakeS() {

        //checks to see if motors are inverted
        topFlywheel.setInverted(Constants.OutakeConstants.topFlywheelReversed);
        bottomFlywheel.setInverted(Constants.OutakeConstants.bottomFlywheelReversed);
        //sets idle mode on motors
        topFlywheel.setIdleMode(IdleMode.kBrake);
        bottomFlywheel.setIdleMode(IdleMode.kBrake);
        //assigns values to encoders
        topFlywheelEncoder = topFlywheel.getEncoder();
        bottomFlywheelEncoder = bottomFlywheel.getEncoder();
        //makes encoders work with the gear ratio (basically means that one turn of the wheel will be one turn of the encoder)
        topFlywheelEncoder.setVelocityConversionFactor(Constants.OutakeConstants.flywheelGearRatio);
        bottomFlywheelEncoder.setVelocityConversionFactor(Constants.OutakeConstants.flywheelGearRatio);
        //get the motor's built in PID settings
        topPIDController = topFlywheel.getPIDController();
        bottomPIDController = bottomFlywheel.getPIDController();
        //set starting PID vals
        kP = Constants.OutakeConstants.PIDConstants.P;
        kI = Constants.OutakeConstants.PIDConstants.I;
        kD = Constants.OutakeConstants.PIDConstants.D;
        kIz = Constants.OutakeConstants.PIDConstants.Iz;
        kFF = Constants.OutakeConstants.PIDConstants.FF;
        kMaxOutput = Constants.OutakeConstants.PIDConstants.max;
        kMinOutput = Constants.OutakeConstants.PIDConstants.min;
        maxVel = Constants.OutakeConstants.PIDConstants.maxVel;
        minVel = Constants.OutakeConstants.PIDConstants.minVel;
        maxAcc = Constants.OutakeConstants.PIDConstants.maxAccel;
        allowedErr = Constants.OutakeConstants.PIDConstants.allowedErr;
        topPIDController.setP(kP);
        topPIDController.setI(kI);
        topPIDController.setD(kD);
        topPIDController.setIZone(kIz);
        topPIDController.setFF(kFF);
        topPIDController.setOutputRange(kMinOutput, kMaxOutput);
        int smartMotionSlot = 0;
        topPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        topPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        topPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        topPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        bottomPIDController.setP(kP);
        bottomPIDController.setI(kI);
        bottomPIDController.setD(kD);
        bottomPIDController.setIZone(kIz);
        bottomPIDController.setFF(kFF);
        bottomPIDController.setOutputRange(kMinOutput, kMaxOutput);
        bottomPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        bottomPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        bottomPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        bottomPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        //sets changes to the motors' controllers
        topFlywheel.burnFlash();
        bottomFlywheel.burnFlash();

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Speed", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Speed", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Average Flywheel Speed", getAverageFlywheelSpeed());
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", Constants.OutakeConstants.PIDConstants.P);
        double i = SmartDashboard.getNumber("I Gain", Constants.OutakeConstants.PIDConstants.I);
        double d = SmartDashboard.getNumber("D Gain", Constants.OutakeConstants.PIDConstants.D);
        double iz = SmartDashboard.getNumber("I Zone", Constants.OutakeConstants.PIDConstants.Iz);
        double ff = SmartDashboard.getNumber("Feed Forward", Constants.OutakeConstants.PIDConstants.FF);
        double max = SmartDashboard.getNumber("Max Output", Constants.OutakeConstants.PIDConstants.max);
        double min = SmartDashboard.getNumber("Min Output", Constants.OutakeConstants.PIDConstants.min);
        double maxV = SmartDashboard.getNumber("Max Velocity", Constants.OutakeConstants.PIDConstants.maxVel);
        double minV = SmartDashboard.getNumber("Min Velocity", Constants.OutakeConstants.PIDConstants.minVel);
        double maxA = SmartDashboard.getNumber("Max Acceleration", Constants.OutakeConstants.PIDConstants.maxAccel);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", Constants.OutakeConstants.PIDConstants.allowedErr);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != kP)) { 
            topPIDController.setP(p); kP = p; 
            bottomPIDController.setP(p); kP = p; 
        }
        if ((i != kI)) {
            topPIDController.setI(i); kI = i; 
            bottomPIDController.setI(i); kI = i; 
        }
        if ((d != kD)) {
            topPIDController.setD(d); kD = d;
            bottomPIDController.setD(d); kD = d;
        }
        if ((iz != kIz)) {
            topPIDController.setIZone(iz); kIz = iz; 
            bottomPIDController.setIZone(iz); kIz = iz; 
        }
        if ((ff != kFF)) {
            topPIDController.setFF(ff); kFF = ff; 
            bottomPIDController.setFF(ff); kFF = ff; 
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) { 
            topPIDController.setOutputRange(min, max); 
            bottomPIDController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        if((maxV != maxVel)) { 
            topPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; 
            bottomPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; 
        }
        if((minV != minVel)) { 
            topPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; 
            bottomPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; 
        }
        if((maxA != maxAcc)) { 
            topPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; 
            bottomPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; 
        }
        if((allE != allowedErr)) { 
            topPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; 
            bottomPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; 
        }
    }

    public static double getAverageFlywheelSpeed() {
        //pulls the speed of the flywheels, used for pid loop
        double speed = topFlywheelEncoder.getVelocity() + bottomFlywheelEncoder.getVelocity();
        speed = speed/2;
        return speed;
    }

    public static double getFlywheelSpeedDifference() {
        return Math.abs(Math.abs(topFlywheelEncoder.getVelocity()) - Math.abs(bottomFlywheelEncoder.getVelocity()));
    }
    public void setRPM(double rpm){
        bottomPIDController.setReference(rpm, CANSparkMax.ControlType.kSmartVelocity);
        topPIDController.setReference(rpm, CANSparkMax.ControlType.kSmartVelocity); //we need to test SmartVelocity
    }
    public void setFF(double FF){
        bottomPIDController.setFF(FF);
        topPIDController.setFF(FF);
    }
    public void setRPMTop(double rpm){
        topPIDController.setReference(rpm, CANSparkMax.ControlType.kSmartVelocity);
    }
    public void setRPMBottom(double rpm){
        bottomPIDController.setReference(rpm, CANSparkMax.ControlType.kSmartVelocity);
    }
    public static double getTopRPMError(double desiredRPM){
        return Math.abs(topFlywheelEncoder.getVelocity()-desiredRPM);
    }
    public static double getBottomRPMError(double desiredRPM){
        return Math.abs(bottomFlywheelEncoder.getVelocity()-desiredRPM);
    }
    /*
     **For shooting amp
     */
    public void setIndividualFlywheelSpeeds(double topWheelSpeed, double bottomWheelSpeed){
        topFlywheel.set(topWheelSpeed);
        bottomFlywheel.set(bottomWheelSpeed);
    }
}
