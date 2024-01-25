package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;

public class IntakeC extends Command {
    private final IntakeS intakeS;

    public IntakeC(IntakeS intakeS) {
        this.intakeS = intakeS;

        addRequirements(intakeS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double intakeSpeed = RobotContainer.driveController.getLeftTriggerAxis();
        double deployIntakeSpeed = 0;

        if (RobotContainer.manipController.getAButtonPressed() == true) {
            deployIntakeSpeed = 0.5;
        } else if (RobotContainer.manipController.getBButtonPressed() == true) {
            deployIntakeSpeed = -0.5;
        }

        intakeS.setPrimaryIntake(intakeSpeed * 1);
        intakeS.deployIntake(deployIntakeSpeed * 1);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
