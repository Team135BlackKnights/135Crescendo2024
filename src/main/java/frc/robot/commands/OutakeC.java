package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;


public class OutakeC extends Command {
   
    private final OutakeS outakeS;

    private final PIDController shooterPID = new PIDController(0.0002, 0, 0);

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
            outakeSpeed = 0.5 + shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), 4000);
        } else if (RobotContainer.manipController.getXButton() == true) {
            outakeSpeed = 0.33 + shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), 2700);
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
