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
       
        double output = RobotContainer.manipController.getLeftY();
        output = Math.abs(output) >= 0.2 ? output : 0;

        hangS.leftHang.set(output); //sets the motors to get the controller values
        hangS.rightHang.set(output); 
    }

    @Override
   
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
