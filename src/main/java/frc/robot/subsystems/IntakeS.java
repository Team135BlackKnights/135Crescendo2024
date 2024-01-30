package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeS extends SubsystemBase {
    boolean intakeReversed = false;
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);
    public RelativeEncoder deployIntakeEncoder;

    public static final DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID); //the intake limit switch

    public IntakeS() {
        primaryIntake.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
        deployIntake.setInverted(Constants.IntakeConstants.deployIntakeReversed);

        primaryIntake.setIdleMode(IdleMode.kBrake);

        deployIntakeEncoder = deployIntake.getEncoder();
        deployIntakeEncoder.setPositionConversionFactor(Constants.IntakeConstants.deployIntakeGearRatio);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Direction", intakeReversed);
    }

    public static boolean noteIsLoaded() {
        return intakeLimitSwitch.get();
    }

    public void setPrimaryIntake(double power) {
        power = power <= 0.1 ? 0.1 : power;
        power = power * (intakeReversed ? -1 : 1);
        primaryIntake.set(power);
    }

    public void deployIntake(double power) {
    
        if (power < 0) {
            if (deployIntakeEncoder.getPosition() < 0) {
                power = 0;
            } else if (deployIntakeEncoder.getPosition() < 3) {
                power = power * 0.5;
            }
        }
        if (power > 0) {
            if (deployIntakeEncoder.getPosition() > 10) {
                power = 0;
            } else if (deployIntakeEncoder.getPosition() > 7) {
                power = power * 0.5;
            }
        }

        deployIntake.set(power);
    }

    public void toggleIntakeDirection() {
        intakeReversed = !intakeReversed;
    }

    public InstantCommand toggleIntakeDirectionCommand() {
        return new InstantCommand(this::toggleIntakeDirection, this);
    }
}
