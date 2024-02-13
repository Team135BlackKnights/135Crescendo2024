package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeS extends SubsystemBase {
    //declarations of motors/encoders and limit switch
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
    public RelativeEncoder deployIntakeEncoder, primaryIntakeEncoder;
    public SparkAbsoluteEncoder absDeployIntakeEncoder;

    public static final DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID); //the intake limit switch

    public IntakeS() {
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
    }

    @Override
    public void periodic() {
        //sets values to SmartDashboard periodically
        SmartDashboard.putNumber("Deploy Intake", deployIntakeEncoder.getPosition());
        SmartDashboard.putBoolean("Note Loaded?", noteIsLoaded());
    }

    public static boolean noteIsLoaded() {
        //checks limit switch to see if note is loaded
        return !intakeLimitSwitch.get();
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
