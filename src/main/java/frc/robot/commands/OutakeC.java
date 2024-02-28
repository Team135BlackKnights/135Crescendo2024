package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;
import frc.robot.Constants;

public class OutakeC extends Command {
    private final OutakeS outakeS;
    private final PIDController shooterPID = new PIDController(0.0001, 0, 0);

    public OutakeC(OutakeS outakeS) {
        this.outakeS = outakeS;

        addRequirements(outakeS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //for amp
        if (RobotContainer.manipController.getRightBumper()){
            
            double topWheelSpeed = 0;
            double bottomWheelSpeed = 0;
            double[] flywheelSpeeds = outakeS.getAverageFlywheelSpeeds();
            topWheelSpeed = Constants.OutakeConstants.idealPercentTop + shooterPID.calculate(flywheelSpeeds[0], 8000*Constants.OutakeConstants.idealPercentTop);
            bottomWheelSpeed = Constants.OutakeConstants.idealPercentBottom + shooterPID.calculate(flywheelSpeeds[1],8000*Constants.OutakeConstants.idealPercentBottom);
            outakeS.setIndividualFlywheelSpeeds(topWheelSpeed, bottomWheelSpeed);
        }
        //for speaker
        else{
            double outakeSpeed = 0;
        if (RobotContainer.manipController.getBButton() == true) {
            outakeSpeed = 1;
        } else if (RobotContainer.manipController.getAButton() == true) {
            outakeSpeed = 0.5 + shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), 4000);
        } else if (RobotContainer.manipController.getXButton() == true) {
            outakeSpeed = 0.33 + shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), 2700);
        }
        outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
    }
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
