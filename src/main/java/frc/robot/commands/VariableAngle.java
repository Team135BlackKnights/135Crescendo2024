package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.subsystems.SwerveS;

public class VariableAngle extends Command {
    private final IntakeS intakeS;
    private final OutakeS outakeS;
    private boolean isFinished = false, isAutonomous;
    Timer timer = new Timer();
    Timer delay = new Timer();

    private final PIDController shooterPID = new PIDController(0.00015, 0, 0);

    private PIDController anglePidController = new PIDController(0.06, 0, 0);

    public VariableAngle(IntakeS intakeS, OutakeS outakeS, boolean isAutonomous) {
        this.intakeS = intakeS;
        this.outakeS = outakeS;
        this.isAutonomous = isAutonomous;

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
        if (delay.get() >=0.5 || Math.abs(RobotContainer.manipController.getRightY()) > 0.2 || RobotContainer.driveController.getLeftTriggerAxis() > 0.1 || RobotContainer.manipController.getLeftTriggerAxis() > 0.1 || RobotContainer.driveController.getRightTriggerAxis() > 0.1 || RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
            isFinished = true;
        }

        double output = anglePidController.calculate(intakeS.getIntakeAngle(), SwerveS.getDesiredShooterAngle());

        if (timer.get() < 0.1 && !isAutonomous) {
            intakeS.setPrimaryIntake(0.5);
        } else if (timer.get() >= 0.15) {
            intakeS.setPrimaryIntake(0);
            double outakeSpeed = 0.91 + shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), 6000);
            outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
        }
        if (RobotContainer.manipController.getLeftBumper() == true) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }
        if (intakeS.intakeWithinBounds() && shooterPID.getPositionError() < 150 && Math.abs(SwerveS.getXError()) < 3 && RobotContainer.manipController.getAButton() == false && Math.abs(output) < 0.1) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }

        
        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("Angle Error", anglePidController.getPositionError());
        SmartDashboard.putNumber("Flywheel Error", shooterPID.getPositionError());

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
