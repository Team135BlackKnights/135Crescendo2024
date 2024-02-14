package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;

public class OutakeC extends Command {
   
    private final OutakeS outakeS;

    public OutakeC(OutakeS outakeS) {
    
        this.outakeS = outakeS;

        addRequirements(outakeS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      
        double outakeSpeed = 0;

        if (RobotContainer.manipController.getBButton() == true) {
            outakeSpeed = 1;
        } else if (RobotContainer.manipController.getAButton() == true) {
            outakeSpeed = 0.5;
        } else if (RobotContainer.manipController.getXButton() == true) {
            outakeSpeed = 0.33;
        } else if (RobotContainer.manipController.getYButton() == true) {
            outakeSpeed = 0.2;
        }

        outakeS.setFiringSpeed(outakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
     
        return false;
    }
}
