package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutakeS extends SubsystemBase {
    public CANSparkMax topFlywheel = new CANSparkMax(Constants.OutakeConstants.topFlywheel, MotorType.kBrushless);
    public CANSparkMax bottomFlywheel = new CANSparkMax(Constants.OutakeConstants.bottomFlywheel, MotorType.kBrushless);

    public RelativeEncoder topFlywheelEncoder;
    public RelativeEncoder bottomFlywheelEncoder;

    public OutakeS() {
        topFlywheel.setInverted(Constants.OutakeConstants.topFlywheelReversed);
        bottomFlywheel.setInverted(Constants.OutakeConstants.bottomFlywheelReversed);

        topFlywheelEncoder = topFlywheel.getEncoder();
        bottomFlywheelEncoder = bottomFlywheel.getEncoder();
    }

    public void setFiringSpeed(double power) {
        topFlywheel.set(power);
        bottomFlywheel.set(power);
    }

    public double getAverageFlywheelSpeed() {
        double speed = topFlywheelEncoder.getVelocity() + bottomFlywheelEncoder.getVelocity();
        speed = speed/2;
        return speed;
    }
}
