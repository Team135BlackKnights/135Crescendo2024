package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        topFlywheel.setIdleMode(IdleMode.kBrake);
        bottomFlywheel.setIdleMode(IdleMode.kBrake);

        topFlywheelEncoder = topFlywheel.getEncoder();
        bottomFlywheelEncoder = bottomFlywheel.getEncoder();

        topFlywheelEncoder.setVelocityConversionFactor(Constants.OutakeConstants.flywheelGearRatio);
        bottomFlywheelEncoder.setVelocityConversionFactor(Constants.OutakeConstants.flywheelGearRatio);

        topFlywheel.burnFlash();
        bottomFlywheel.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Speed", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Speed", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Average Flywheel Speed", getAverageFlywheelSpeed());
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
