package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutakeS extends SubsystemBase {
    public CANSparkMax leftFlywheel = new CANSparkMax(Constants.OutakeConstants.leftFlywheel, MotorType.kBrushless);
    public CANSparkMax rightFlywheel = new CANSparkMax(Constants.OutakeConstants.rightFlywheel, MotorType.kBrushless);

    public OutakeS() {
        leftFlywheel.setInverted(Constants.OutakeConstants.leftFlywheelReversed);
        rightFlywheel.setInverted(Constants.OutakeConstants.rightFlywheelReversed);
    }

    public void setFiringSpeed(double power) {
        leftFlywheel.set(power);
        rightFlywheel.set(power);
    }
}
