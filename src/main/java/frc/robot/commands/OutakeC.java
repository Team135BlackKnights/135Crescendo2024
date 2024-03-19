package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;
import frc.robot.Constants.OutakeConstants;

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
            double[] flywheelSpeeds = OutakeS.getFlywheelSpeeds();
            double topWheelSpeed = OutakeConstants.idealPercentTop + shooterPID.calculate(flywheelSpeeds[0], OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentTop);
            double bottomWheelSpeed = OutakeConstants.idealPercentBottom + shooterPID.calculate(flywheelSpeeds[1],OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentBottom);
            outakeS.setIndividualFlywheelSpeeds(topWheelSpeed, bottomWheelSpeed);
        }
        //for speaker
        else{
            double outakeSpeed = 0;
            if (RobotContainer.driveController.getLeftBumper() == true) {
            outakeSpeed = -0.5;
        }
        if (RobotContainer.manipController.getAButton() == true) {
            outakeSpeed = 0.5 + shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 4000);
        } else if (RobotContainer.manipController.getXButton() == true) {
            outakeSpeed = 0.33 + shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 2700);
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
