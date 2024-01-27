package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeS extends SubsystemBase {
    boolean intakeReversed = false;
    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax deployIntake = new CANSparkMax(Constants.IntakeConstants.deployIntakeID, MotorType.kBrushless);

    public static final DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID); //the intake limit switch

    public IntakeS() {
        primaryIntake.setInverted(Constants.IntakeConstants.lowerPrimaryIntakeReversed);
        deployIntake.setInverted(Constants.IntakeConstants.feederIntakeReversed);
    }

    public static boolean noteIsLoaded() {
        return intakeLimitSwitch.get();
    }

    public void setPrimaryIntake(double power) {
        power = power <= 0.1 ? 0.1 : power;
        power = intakeReversed ? -1 * power : power;
        primaryIntake.set(power);
    }

    public void deployIntake(double power) {
    
        deployIntake.set(power);
        
    }

    public void toggleIntakeDirection() {
        intakeReversed = !intakeReversed;
    }

    public InstantCommand toggleIntakeDirectionCommand() {
        return new InstantCommand(this::toggleIntakeDirection, this);
    }
}
