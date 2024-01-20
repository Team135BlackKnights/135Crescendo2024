package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;

public class Rotate extends Command {
    double desiredAngle;
    SwerveS swerveS;
    boolean isFinished = false;

    public Rotate(SwerveS swerveS, double desiredAngle) {
        this.desiredAngle = desiredAngle;
        this.swerveS = swerveS;
        addRequirements(swerveS);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
