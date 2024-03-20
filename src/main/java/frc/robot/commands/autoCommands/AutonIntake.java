package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;

public class AutonIntake extends Command {
    private final IntakeS intakeS;
    private boolean isFinished = false;
    Timer timer = new Timer();
    Timer delayTimer = new Timer();
    
    public AutonIntake(IntakeS intakeS) {
        this.intakeS = intakeS;
        addRequirements(intakeS);
    }

    @Override
    public void initialize() {
        isFinished = false;
        delayTimer.reset();
        delayTimer.start();
    }

    @Override
    public void execute() {
        IntakeS.detected = IntakeS.colorSensorV3.getColor();
        IntakeS.colorMatchResult = IntakeS.colorMatch.matchClosestColor(IntakeS.detected);

        if (timer.get() > 0.125) {
            isFinished = true;
        }
        if (timer.get() > 0) {
            intakeS.setPrimaryIntake(0.25);
        } else {
            intakeS.setPrimaryIntake(-0.5);
        }
        //runs intake if note is not loaded
        if (IntakeS.noteIsLoaded() && delayTimer.get() > 0.5) {
            delayTimer.stop();
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeS.setPrimaryIntake(0);
        timer.stop();
        timer.reset();
        delayTimer.stop();
        delayTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
