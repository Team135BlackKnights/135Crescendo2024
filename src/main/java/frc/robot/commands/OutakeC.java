package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;


public class OutakeC extends Command {
    private final OutakeS outakeS;
    double outakeSpeed;
    public OutakeC(OutakeS outakeS) {
        this.outakeS = outakeS;

        addRequirements(outakeS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (RobotContainer.manipController.getAButton() && RobotContainer.manipController.getStartButton() && !OutakeS.runningTest){
            outakeS.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        }
        else if (RobotContainer.manipController.getBButton() && RobotContainer.manipController.getStartButton() && !OutakeS.runningTest){
            outakeS.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        }
        else if (RobotContainer.manipController.getXButton() && RobotContainer.manipController.getStartButton() && !OutakeS.runningTest){
            outakeS.sysIdDynamic(SysIdRoutine.Direction.kForward);
        }
        else if (RobotContainer.manipController.getYButton() && RobotContainer.manipController.getStartButton() && !OutakeS.runningTest){
            outakeS.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        }
        //for amp
        if (RobotContainer.manipController.getRightBumper()){
            double topWheelSpeed = 2414; // 34%
            double bottomWheelSpeed = 2201; //31%
            //double topWheelSpeed = OutakeConstants.idealPercentTop + outakeS.shooterPID.calculate(OutakeS.topFlywheelEncoder.getVelocity(), OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentTop);
            //double bottomWheelSpeed = OutakeConstants.idealPercentBottom + outakeS.shooterPID.calculate(OutakeS.bottomFlywheelEncoder.getVelocity(),OutakeConstants.flywheelMaxRPM*OutakeConstants.idealPercentBottom);
            outakeS.setIndividualFlywheelSpeeds(topWheelSpeed, bottomWheelSpeed);
        }
        //for speaker
        else{
            if (RobotContainer.driveController.getLeftBumper() == true) {
                outakeSpeed = -1775; //was -.25
            }
            if (RobotContainer.manipController.getAButton() && !RobotContainer.manipController.getStartButton()) {
                outakeSpeed = 4000; // was outakeSpeed = 0.49 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 4000),-0.1,0.1);   
            } else if (RobotContainer.manipController.getXButton() && !RobotContainer.manipController.getStartButton()) {
                outakeSpeed = 2700; // was outakeSpeed = 0.355 + MathUtil.clamp(outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 2700), -0.1, 0.1);
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
