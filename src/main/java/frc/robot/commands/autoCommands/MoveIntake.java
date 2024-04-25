package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeS;

public class MoveIntake extends Command {
    //variable declaration, new timer made, requirements added
    boolean isFinished = false;
    IntakeS intakeS;

    public MoveIntake(IntakeS intakeS){
        this.intakeS = intakeS;
        addRequirements(intakeS);
    }

    public void initialize(){
        //resets and starts the timer upon command being called
        isFinished = false;
    }

    @Override
    public void execute(){
        //intake deployed at full power for a certain amount of time
        intakeS.deployIntake(1);
        if (IntakeS.getIntakePosition() >= IntakeConstants.deployIntakeOuterBound) {
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