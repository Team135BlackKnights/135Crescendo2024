package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeS extends SubsystemBase {
    boolean intakeReversed = false;

    public CANSparkMax upperPrimaryIntake = new CANSparkMax(Constants.IntakeConstants.upperPrimaryIntakeID, MotorType.kBrushless);
    public CANSparkMax lowerPrimaryIntake = new CANSparkMax(Constants.IntakeConstants.lowerPrimaryIntakeID, MotorType.kBrushless);
    public CANSparkMax feederIntake = new CANSparkMax(Constants.IntakeConstants.feederIntakeID, MotorType.kBrushless);

    public IntakeS() {
        upperPrimaryIntake.setInverted(Constants.IntakeConstants.upperPrimaryIntakeReversed);
        lowerPrimaryIntake.setInverted(Constants.IntakeConstants.lowerPrimaryIntakeReversed);
        feederIntake.setInverted(Constants.IntakeConstants.feederIntakeReversed);
    }

    public void setPrimaryIntake(double power) {
        power = power <= 0.1 ? 0.1 : power;
        power = intakeReversed ? -1 * power : power;
        upperPrimaryIntake.set(power);
        lowerPrimaryIntake.set(power);
    }

    public void setFeederIntake(double power) {
        power = intakeReversed ? -1 * power : power;
        feederIntake.set(power);
    }

    public void toggleIntakeDirection() {
        intakeReversed = !intakeReversed;
    }

    public InstantCommand toggleIntakeDirectionCommand() {
        return new InstantCommand(this::toggleIntakeDirection, this);
    }
}
