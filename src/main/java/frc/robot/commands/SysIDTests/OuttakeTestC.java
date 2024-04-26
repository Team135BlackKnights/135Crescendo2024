package frc.robot.commands.SysIDTests;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;
import frc.robot.Constants.DataLog;
public class OuttakeTestC extends Command{
    OutakeS outakeS;
    private static Timer timer = new Timer();
    private static boolean isFinished;
    public OuttakeTestC(OutakeS outake){
        outakeS = outake;
        addRequirements(outakeS);
    }
    @Override
    public void initialize() {
        timer.start();
        isFinished = false;
        OutakeS.SysIDTestRunning = true;
    }
    @Override
    public void execute(){
    if (timer.get() < DataLog.testRunSeconds){
        if (RobotContainer.manipController.getAButton()){
            outakeS.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        }
        else if (RobotContainer.manipController.getBButton()){
            outakeS.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        }
        else if (RobotContainer.manipController.getXButton()){
            outakeS.sysIdDynamic(SysIdRoutine.Direction.kForward);
        }
        else if (RobotContainer.manipController.getYButton()){
            outakeS.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        }
        //for amp
        else{
            isFinished = true;
        }
    }
    else{
        isFinished = true;
    }
    }
    @Override 
    public void end(boolean interrupted){
        timer.stop();
        timer.reset();
        OutakeS.SysIDTestRunning = false;
    }
    public boolean isFinished(){
        return isFinished;
    }
}
