package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;
import frc.robot.Constants.OutakeConstants;

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
        //for amp
        if (RobotContainer.manipController.getRightBumper()){
            outakeS.setRPMTop(OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentTop);
            outakeS.setRPMBottom(OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentBottom);
        }
        //for speaker
        else{
            double outakeSpeed = 0;
            if (RobotContainer.driveController.getLeftBumper() == true) {
            outakeSpeed = -0.25;
            outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
        }
        if (RobotContainer.manipController.getAButton() == true) {
          //  outakeS.setFF(.49); //may not be needed.
            outakeS.setRPM(4000);
        } else if (RobotContainer.manipController.getXButton() == true) {
            //  outakeS.setFF(.355); //may not be needed.
            outakeS.setRPM(2700);
        }
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
