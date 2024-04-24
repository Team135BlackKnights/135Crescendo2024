package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.Constants.DataLog;

public class VariableAngle extends Command {
    private final IntakeS intakeS;
    private final OutakeS outakeS;
    private int desiredRPM;
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

        double output = intakeS.anglePidController.calculate(intakeS.getIntakeAngle(), CameraS.getDesiredShooterAngle());

        if (timer.get() < 0.15 && !isAutonomous) {
            intakeS.setPrimaryIntake(0.2);
        } else if (timer.get() >= 0.25 && Math.abs(intakeS.anglePidController.getPositionError()) < 10) {
            intakeS.setPrimaryIntake(0);
            if (CameraS.getDistanceFromSpeakerUsingRobotPose() > 4.5) {
            //  outakeS.setFF(.85); //may not be needed.
                desiredRPM = 6000;
            } else if (CameraS.getDistanceFromSpeakerUsingRobotPose() > 2.4) {
            //  outakeS.setFF(.67); //may not be needed.
                desiredRPM = 4750;
            } else {
            //  outakeS.setFF(.46); //may not be needed.
                desiredRPM = 3300;
            }
            outakeS.setIndividualFlywheelSpeeds(desiredRPM, desiredRPM);
        }
        if (RobotContainer.manipController.getLeftBumper()) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }

        if (OutakeS.getFlywheelSpeedDifference() < 100 && timer.get() >= 0.3 && (intakeS.intakeWithinBounds() || Math.abs(intakeS.anglePidController.getPositionError()) < 0.5) && outakeS.shooterPID.getPositionError() < 150 && Math.abs(SwerveS.getXError()) < 3 && !RobotContainer.manipController.getAButton() && Math.abs(output) < 0.1) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }

        
      //  SmartDashboard.putNumber("Angle Output", output);
        SmartDashboard.putNumber("Angle Error", intakeS.anglePidController.getPositionError());
        SmartDashboard.putNumber("Flywheel Top", OutakeS.topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Flywheel Bottom ", OutakeS.bottomFlywheelEncoder.getVelocity());
        if (delay.get() < 0.2) {
            DataLog.angleOutputDegrees = intakeS.getIntakeAngle();
            DataLog.variableAngleDistance = CameraS.getDistanceFromSpeakerUsingRobotPose();
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
