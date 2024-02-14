package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HangS;

public class HangC extends Command {
    private final HangS hangS;
    
    public HangC(HangS hangS){
        this.hangS = hangS;
        
        addRequirements(hangS);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        hangS.leftHang.set(RobotContainer.manipController.getLeftY()); //sets the motors to get the controller values
        hangS.rightHang.set(RobotContainer.manipController.getLeftY()); 
    }

    @Override
    public void end(boolean x){

    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
