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
        if (RobotContainer.manipController.rightBumper().getAsBoolean()){
            double topWheelSpeed = OutakeConstants.idealPercentTop + outakeS.shooterPID.calculate(OutakeS.topFlywheelEncoder.getVelocity(), OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentTop);
            double bottomWheelSpeed = OutakeConstants.idealPercentBottom + outakeS.shooterPID.calculate(OutakeS.bottomFlywheelEncoder.getVelocity(),OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentBottom);
            outakeS.setIndividualFlywheelSpeeds(topWheelSpeed, bottomWheelSpeed);
        }
        //for speaker
        else{
            double outakeSpeed = 0;
            if (RobotContainer.driveController.getLeftBumper() == true) {
            outakeSpeed = -0.25;
        }
        if (RobotContainer.manipController.a().getAsBoolean() == true) {
            outakeSpeed = 0.49 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 4000),-0.1,0.1);
        } else if (RobotContainer.manipController.x().getAsBoolean() == true) {
            outakeSpeed = 0.355 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 2700), -0.1, 0.1);
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
