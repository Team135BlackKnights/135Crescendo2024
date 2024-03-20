package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutakeS extends SubsystemBase {
    //motor declarations
   
    public CANSparkMax topFlywheel = new CANSparkMax(Constants.OutakeConstants.topFlywheel, MotorType.kBrushless);
    public CANSparkMax bottomFlywheel = new CANSparkMax(Constants.OutakeConstants.bottomFlywheel, MotorType.kBrushless);
    //encoder declarations
    public static RelativeEncoder 
    topFlywheelEncoder,
    bottomFlywheelEncoder;



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
        //sets changes to the motors' controllers
        topFlywheel.burnFlash();
        bottomFlywheel.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Speed", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Speed", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Average Flywheel Speed", getAverageFlywheelSpeed());
    }

    public static double getAverageFlywheelSpeed() {
        //pulls the speed of the flywheels, used for pid loop
        double speed = topFlywheelEncoder.getVelocity() + bottomFlywheelEncoder.getVelocity();
        speed = speed/2;
        return speed;
    }
    public static double[] getFlywheelSpeeds(){
        return new double[]{topFlywheelEncoder.getVelocity(), bottomFlywheelEncoder.getVelocity()};
    }
    
    /*
     **For shooting amp
     */
    public void setIndividualFlywheelSpeeds(double topWheelSpeed, double bottomWheelSpeed){
        topFlywheel.set(topWheelSpeed);
        bottomFlywheel.set(bottomWheelSpeed);
    }
}
