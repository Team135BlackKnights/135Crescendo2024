package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
<<<<<<< Updated upstream
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.HangConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.HangConstants;

>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HangS extends SubsystemBase {
  
    //hang motors
    public CANSparkMax leftHang = new CANSparkMax(HangConstants.leftHangID, MotorType.kBrushless);
    public CANSparkMax rightHang = new CANSparkMax(HangConstants.rightHangID, MotorType.kBrushless);
<<<<<<< Updated upstream
    public RelativeEncoder leftHangEncoder, rightHangEncoder;
=======
>>>>>>> Stashed changes
    
    //sets hang idle mode & saves it
    public HangS(){
    
<<<<<<< Updated upstream
        leftHang.setIdleMode(IdleMode.kCoast);
        rightHang.setIdleMode(IdleMode.kCoast);
=======
        leftHang.setIdleMode(IdleMode.kBrake);
        rightHang.setIdleMode(IdleMode.kBrake);
>>>>>>> Stashed changes

        leftHang.setInverted(HangConstants.leftHangReversed);
        rightHang.setInverted(HangConstants.rightHangReversed);

<<<<<<< Updated upstream
        leftHangEncoder = leftHang.getEncoder();
        rightHangEncoder = rightHang.getEncoder();

        leftHang.burnFlash();
        rightHang.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Hang", leftHangEncoder.getPosition());
        SmartDashboard.putNumber("Right Hang", rightHangEncoder.getPosition());
    }
=======
        leftHang.burnFlash();
        rightHang.burnFlash();
    }
>>>>>>> Stashed changes
}
