package frc.robot.subsystems;

import frc.robot.sendables.HangSendable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.HangConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class HangS extends SubsystemBase {
    HangSendable hangSendable = new HangSendable();
    //hang motors
    public CANSparkMax leftHang = new CANSparkMax(HangConstants.leftHangID, MotorType.kBrushless);
    public CANSparkMax rightHang = new CANSparkMax(HangConstants.rightHangID, MotorType.kBrushless);
    public static RelativeEncoder leftHangEncoder, rightHangEncoder;
    
    public static DoubleSupplier 
    leftHangSupplier = () ->leftHangEncoder.getPosition(),
    rightHangSupplier = () -> rightHangEncoder.getPosition();

    //sets hang idle mode & saves it
    public HangS(){
    
        leftHang.setIdleMode(IdleMode.kBrake);
        rightHang.setIdleMode(IdleMode.kBrake);

        leftHang.setInverted(HangConstants.leftHangReversed);
        rightHang.setInverted(HangConstants.rightHangReversed);

        leftHangEncoder = leftHang.getEncoder();
        rightHangEncoder = rightHang.getEncoder();

        leftHang.burnFlash();
        rightHang.burnFlash();
        
        SmartDashboard.putData(hangSendable);
    }


}
