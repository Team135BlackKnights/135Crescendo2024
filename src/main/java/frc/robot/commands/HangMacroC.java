package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HangConstants;
public class HangMacroC extends Command{
    private final HangS subsystem;
    public static boolean isFinished;
    public double 
    leftHangSetpoint,
    rightHangSetpoint,
    deadband;
    public int hangState;
    public static PIDController
    leftHangController = new PIDController(.00005, 0, 0),
    rightHangController = new PIDController(.00005, 0, 0);
    
    public HangMacroC(HangS hang, int hangState){
        this.subsystem = hang;
        this.hangState = hangState;
        addRequirements(subsystem);
        
        deadband = 1;
    }
    @Override
    public void initialize(){
        isFinished = false;
        leftHangSetpoint = HangConstants.hangMacroStates[hangState][0];
        rightHangSetpoint = HangConstants.hangMacroStates[hangState][1];
    }
    @Override
    public void execute(){
        SmartDashboard.setDefaultNumber("Hang State", hangState);
        if (Math.abs(leftHangController.getPositionError()) < deadband && Math.abs(rightHangController.getPositionError()) < deadband){
            isFinished = true;
        }
        else{
           
            HangS.setHangMotors(leftHangController.calculate(HangS.leftHangEncoder.getPosition(), leftHangSetpoint), rightHangController.calculate(HangS.rightHangEncoder.getPosition(), rightHangSetpoint));
        }
       
    }
    @Override
    public void end(boolean interrupted){
        HangS.leftHang.set(0);
        HangS.rightHang.set(0);
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
