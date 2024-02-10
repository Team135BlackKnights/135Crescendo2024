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
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
    public RelativeEncoder deployIntakeEncoder, primaryIntakeEncoder;
    public SparkAbsoluteEncoder absDeployIntakeEncoder;

    public static final DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID); //the intake limit switch

    public IntakeS() {
        primaryIntake.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
        deployIntake.setInverted(Constants.IntakeConstants.deployIntakeReversed);

        primaryIntake.setIdleMode(IdleMode.kBrake);

        primaryIntakeEncoder = primaryIntake.getEncoder();
        primaryIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);
        primaryIntakeEncoder.setVelocityConversionFactor(Constants.IntakeConstants.primaryIntakeGearRatio);

        deployIntakeEncoder = deployIntake.getEncoder();
        deployIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.deployIntakeGearRatio);

        absDeployIntakeEncoder = deployIntake.getAbsoluteEncoder(Type.kDutyCycle);

        primaryIntake.burnFlash();
        deployIntake.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Deploy Intake", deployIntakeEncoder.getPosition());
        SmartDashboard.putBoolean("Note Loaded?", noteIsLoaded());
    }

    public static boolean noteIsLoaded() {
        return !intakeLimitSwitch.get();
    }

    public void setPrimaryIntake(double power) {
        //power = power <= 0.1 ? 0.1 : power;
        primaryIntake.set(power);
    }

    public void deployIntake(double power) {
        if (power < 0) {
            if (deployIntakeEncoder.getPosition() < 0) {
                power = 0;
            } else if (deployIntakeEncoder.getPosition() < 44.1) {
                power = power * 0.5;
            }
        }
        if (power > 0) {
            if (deployIntakeEncoder.getPosition() > 113) {
                power = 0;
            } else if (deployIntakeEncoder.getPosition() > 79) {
                power = power * 0.5;
            }
        }

        SmartDashboard.putNumber("Deploy Intake Percentage", power);

        deployIntake.set(power);
    }
}
