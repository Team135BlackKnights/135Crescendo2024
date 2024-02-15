package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HangS;
import frc.robot.subsystems.SwerveS;

public class HangC extends Command {
    private final HangS hangS;
    private final SwerveS swerveS;

    private PIDController hangPidController = new PIDController(0.1, 0, 0);
    
    public HangC(HangS hangS, SwerveS swerveS){
        this.hangS = hangS;
        this.swerveS = swerveS;
        
        addRequirements(hangS, swerveS);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
       
        double output = RobotContainer.manipController.getLeftY();
        output = Math.abs(output) >= 0.2 ? output : 0;

        double hangAdjustment = hangPidController.calculate(swerveS.getTilt(), 0);

        hangS.leftHang.set(output+hangAdjustment); //sets the motors to get the controller values
        hangS.rightHang.set(output-hangAdjustment); 
    }

    @Override
   
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
