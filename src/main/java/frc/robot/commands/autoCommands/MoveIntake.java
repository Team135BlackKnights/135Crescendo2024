package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;

public class MoveIntake extends Command {
    //variable declaration, new timer made, requirements added
    boolean isFinished = false;
    IntakeS intakeS;
    double time;
    Timer timer = new Timer();
    public MoveIntake(IntakeS intakeS, double time){
        this.intakeS = intakeS;
        this.time = time;
        addRequirements(intakeS);
    }

    public void initialize(){
        //resets and starts the timer upon command being called
        isFinished = false;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        //intake deployed at full power for a certain amount of time
        intakeS.deployIntake(1);
        if (timer.get() > time) {
            isFinished = true;
        }
    }
    
    
    @Override
    public void end(boolean interrupted){
        intakeS.deployIntake(0);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}