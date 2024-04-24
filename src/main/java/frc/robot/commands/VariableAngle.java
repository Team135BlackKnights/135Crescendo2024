package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.subsystems.SwerveS;
import frc.robot.Constants.DataLog;

public class VariableAngle extends Command {
    private final IntakeS intakeS;
    private final OutakeS outakeS;
    private boolean isFinished = false, isAutonomous;
    Timer timer = new Timer();
    Timer delay = new Timer();

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
        if (delay.get() >=0.75 || Math.abs(RobotContainer.manipController.getRightY()) > 0.2 || RobotContainer.driveController.getLeftTriggerAxis() > 0.1 || RobotContainer.manipController.getLeftTriggerAxis() > 0.1 || RobotContainer.driveController.getRightTriggerAxis() > 0.1 || RobotContainer.manipController.getRightTriggerAxis() > 0.1) {
            isFinished = true;
        }

        double output = intakeS.anglePidController.calculate(intakeS.getIntakeAngle(), SwerveS.getDesiredShooterAngle());

        if (timer.get() < 0.15 && !isAutonomous) {
            intakeS.setPrimaryIntake(0.2);
        } else if (timer.get() >= 0.25 && Math.abs(intakeS.anglePidController.getPositionError()) < 10) {
            intakeS.setPrimaryIntake(0);
            double outakeSpeed;
            if (SwerveS.getDistanceFromSpeakerUsingRobotPose() > 4.5) {
                outakeSpeed = 0.85 + outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 6000);
            } else if (SwerveS.getDistanceFromSpeakerUsingRobotPose() > 2.4) {
                outakeSpeed = 0.67 + outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 4750);
            } else {
                outakeSpeed  = 0.46 + outakeS.shooterPID.calculate(OutakeS.getAverageFlywheelSpeed(), 3300);
            }
            outakeS.setIndividualFlywheelSpeeds(outakeSpeed, outakeSpeed);
        }
        if (RobotContainer.manipController.leftBumper().getAsBoolean()) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }
        if (OutakeS.getFlywheelSpeedDifference() < 100 && timer.get() >= 0.3 && (intakeS.intakeWithinBounds() || Math.abs(intakeS.anglePidController.getPositionError()) < 0.5) && outakeS.shooterPID.getPositionError() < 150 && Math.abs(SwerveS.getXError()) < 3 && RobotContainer.manipController.a().getAsBoolean() == false && Math.abs(output) < 0.1) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }

        
        SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("Angle Error", intakeS.anglePidController.getPositionError());
        SmartDashboard.putNumber("Flywheel Error", outakeS.shooterPID.getPositionError());
        if (delay.get() < 0.2) {
            DataLog.angleOutputDegrees = intakeS.getIntakeAngle();
            DataLog.variableAngleDistance = SwerveS.getDistanceFromSpeakerUsingRobotPose();
        }
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
