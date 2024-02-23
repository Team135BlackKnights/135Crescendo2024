package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;

public class AutonIntake extends Command {
    private final IntakeS intakeS;
    private boolean isFinished = false;
    
    public AutonIntake(IntakeS intakeS) {
        this.intakeS = intakeS;
        addRequirements(intakeS);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        //runs intake if note is not loaded
        if (IntakeS.noteIsLoaded()) {
            isFinished = true;
        }
        intakeS.setPrimaryIntake(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        intakeS.setPrimaryIntake(0);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
