package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeS extends SubsystemBase {
    boolean intakeReversed = false;

    public CANSparkMax primaryIntake = new CANSparkMax(Constants.IntakeConstants.primaryIntakeID, MotorType.kBrushless);
    public CANSparkMax feederIntake = new CANSparkMax(Constants.IntakeConstants.feederIntakeID, MotorType.kBrushless);

    public IntakeS() {
        primaryIntake.setInverted(Constants.IntakeConstants.primaryIntakeReversed);
        feederIntake.setInverted(Constants.IntakeConstants.feederIntakeReversed);
    }

    public void setPrimaryIntake(double power) {
        power = power <= 0.1 ? 0.1 : power;
        power = intakeReversed ? -1 * power : power;
        primaryIntake.set(power);
    }

    public void setFeederIntake(double power) {
        power = power <= 0.1 ? 0.1 : power;
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
