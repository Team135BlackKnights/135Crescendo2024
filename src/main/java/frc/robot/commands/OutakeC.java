package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutakeS;

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
        double outakeSpeed = RobotContainer.driveController.getRightTriggerAxis();

        outakeS.setFiringSpeed(outakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
