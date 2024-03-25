package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;

public class SetAngle extends Command {
    private final IntakeS intakeS;
    private final OutakeS outakeS;
    private boolean isFinished = false;
    private double desAngle;
    Timer timer = new Timer();
    Timer delay = new Timer();

    public SetAngle(IntakeS intakeS, OutakeS outakeS, double desAngle) {
        this.intakeS = intakeS;
        this.outakeS = outakeS;
        this.desAngle = desAngle;

        addRequirements(intakeS, outakeS);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        delay.reset();
        isFinished = false;
    }

    @Override
    public void execute() {
        if (delay.get() >=1 || Math.abs(RobotContainer.manipController.getRightY()) > 0.2 || RobotContainer.driveController.getLeftTriggerAxis() > 0.1 || RobotContainer.manipController.getLeftTriggerAxis() > 0.1 || RobotContainer.driveController.getRightTriggerAxis() > 0.1 || RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
            isFinished = true;
        }

        double output = intakeS.anglePidController.calculate(intakeS.getIntakeAngle(), desAngle);

        if (timer.get() < 0.15) {
            intakeS.setPrimaryIntake(0.2);
        } else if (timer.get() >= 0.25  && Math.abs(intakeS.anglePidController.getPositionError()) < 10) {
            intakeS.setPrimaryIntake(0);
            double outakeSpeed = 0.91 + outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 6000);
            outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
        }
        if (RobotContainer.manipController.getLeftBumper() == true) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }
        if (OutakeS.getFlywheelSpeedDifference() < 100 && timer.get() >= 0.3 && outakeS.shooterPID.getPositionError() < 150 && RobotContainer.manipController.getAButton() == false && Math.abs(output) < 0.1) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }

        
        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("Angle Error", intakeS.anglePidController.getPositionError());
        SmartDashboard.putNumber("Flywheel Error", outakeS.shooterPID.getPositionError());

        intakeS.deployIntake(output);
    }

    @Override
    public void end(boolean interrupted) {
        intakeS.deployIntake(0);
        intakeS.setPrimaryIntake(0);
        outakeS.setIndividualFlywheelSpeeds(0, 0);
        timer.stop();
        timer.reset();
        delay.stop();
        delay.reset();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
