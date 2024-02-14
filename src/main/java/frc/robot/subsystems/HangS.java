package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.HangConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HangS extends SubsystemBase {
    //hang motors
    public CANSparkMax leftHang = new CANSparkMax(HangConstants.leftHangID, MotorType.kBrushless);
    public CANSparkMax rightHang = new CANSparkMax(HangConstants.rightHangID, MotorType.kBrushless);
    //sets hang idle mode & saves it
    public HangS(){
        leftHang.setIdleMode(IdleMode.kBrake);
        rightHang.setIdleMode(IdleMode.kBrake);

        leftHang.setInverted(HangConstants.leftHangReversed);
        rightHang.setInverted(HangConstants.rightHangReversed);

        leftHang.burnFlash();
        rightHang.burnFlash();
    }
}
