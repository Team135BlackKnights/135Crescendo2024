package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.SwerveS;

public class VariableAngle extends Command {
    private final IntakeS intakeS;
    private boolean isFinished = false;

    private PIDController anglePidController = new PIDController(0.06, 0, 0);

    public VariableAngle(IntakeS intakeS) {
        this.intakeS = intakeS;

        addRequirements(intakeS);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        if (Math.abs(RobotContainer.manipController.getRightY()) > 0.2 || RobotContainer.manipController.getLeftBumper() == true || RobotContainer.driveController.getLeftTriggerAxis() > 0.1 || RobotContainer.manipController.getLeftTriggerAxis() > 0.1 || RobotContainer.driveController.getRightTriggerAxis() > 0.1 || RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
            isFinished = true;
        }

        double output = anglePidController.calculate(intakeS.getIntakeAngle(), SwerveS.getDesiredShooterAngle());

        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("Angle Error", anglePidController.getPositionError());

        intakeS.deployIntake(output);
    }

    @Override
    public void end(boolean interrupted) {
        intakeS.deployIntake(0);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
