package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;




public class IntakeS extends SubsystemBase {
    //declarations of motors/encoders and limit switch
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
    public RelativeEncoder deployIntakeEncoder, primaryIntakeEncoder;
    public DutyCycleEncoder absDeployIntakeEncoder;
    public static DigitalInput colorSensorInput = new DigitalInput(IntakeConstants.colorSensorPort);
    public static Thread sensorThread;
    public static int timesRan;
    //Note: See if we can do block declaration on these
    public static DoubleSupplier deployIntakeSupplier = () -> deployIntakeEncoder.getPosition();
    public static DoubleSupplier getIntakePositionSupplier = () -> getIntakePosition();
    public static BooleanSupplier noteIsLoadedSupplier = () -> noteIsLoaded();
    public static DoubleSupplier intakeAngleSupplier = () -> getIntakeAngle();
    public static BooleanSupplier intakeWithinBoundsSupplier = () -> intakeWithinBounds();
    public IntakeS() {
        timesRan = 0;


        //sets intake motors to reversed, sets idleMode to brake
        primaryIntake.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
        deployIntake.setInverted(Constants.IntakeConstants.deployIntakeReversed);

        primaryIntake.setIdleMode(IdleMode.kBrake);
        //creates encoders and makes them work with the gear ratios
        primaryIntakeEncoder = primaryIntake.getEncoder();
        primaryIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);
        primaryIntakeEncoder.setVelocityConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);

        deployIntakeEncoder = deployIntake.getEncoder();

        absDeployIntakeEncoder = new DutyCycleEncoder(Constants.IntakeConstants.intakeAbsEncoderID);
        //sets changes to motor (resource intensive, ONLY CALL ON INITIALIZATION)
        primaryIntake.burnFlash();
        deployIntake.burnFlash();
        

        //Color sensor thread

    }
    @Override
    public void periodic() {
        
        //sets values to SmartDashboard periodically
        SmartDashboard.putNumber("Deploy Intake", deployIntakeEncoder.getPosition());
        SmartDashboard.putNumber("Deploy Intake Abs", getIntakePosition());
        SmartDashboard.putBoolean("Note Loaded?", noteIsLoaded());
        SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
        SmartDashboard.putBoolean("Intake Within Bounds", intakeWithinBounds());

    }

    public double getIntakePosition() {
        return absDeployIntakeEncoder.getAbsolutePosition()*Constants.IntakeConstants.absIntakeEncoderConversionFactor - Constants.IntakeConstants.absIntakeEncoderOffset;
    }

    public double getIntakeAngle() {
        return getIntakePosition()-42;
    }

    public boolean intakeWithinBounds() {
        return getIntakeAngle() > SwerveS.getDesiredShooterLowerBound() && getIntakeAngle() < SwerveS.getDesiredShooterUpperBound() || (Math.abs(getIntakeAngle()-SwerveS.getDesiredShooterAngle()) < 0.75);
    }
    

    public static boolean noteIsLoaded() {
        //pulls data from beam break sensor
        return colorSensorInput.get();
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
        SmartDashboard.putNumber("Deploy Intake Percentage", power);

        deployIntake.set(power);
    }
    }