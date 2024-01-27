package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;

public class MoveIntake extends Command {
    
    boolean isFinished = false;
    IntakeS intakeS;
    public MoveIntake(IntakeS intakeS){
        this.intakeS = intakeS;
        addRequirements(intakeS);
    }

    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){
        if (!IntakeS.intakeLimitSwitch.get()){
            intakeS.deployIntake(.5);
        }else{
            isFinished = true;
        }
       
    }
    
    
    @Override
    public void end(boolean interrupted){
        intakeS.deployIntake(0);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}