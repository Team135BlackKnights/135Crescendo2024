package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;
import frc.robot.Constants.IntakeConstants;

public class VariableSpeed extends Command {
    private final IntakeS intakeS;
    private final OutakeS outakeS;
    private boolean isFinished = false, isAutonomous;
    Timer timer = new Timer();
    Timer delay = new Timer();

    public VariableSpeed(IntakeS intakeS, OutakeS outakeS, boolean isAutonomous) {
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

        if (timer.get() < 0.15 && !isAutonomous) {
            intakeS.setPrimaryIntake(0.2);
        } else if (timer.get() >= 0.25) {
            intakeS.setPrimaryIntake(0);
        //  outakeS.setFF(.49); //may not be needed.
            outakeS.setRPM(4000);
        }
        if (RobotContainer.manipController.getLeftBumper() == true) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }
        if (CameraS.robotInRange() && OutakeS.getBottomRPMError(4000) < 100 && OutakeS.getTopRPMError(4000) < 100 && timer.get() >= 0.3 && Math.abs(CameraS.getXError()) < 3 && RobotContainer.manipController.getAButton() == false && IntakeS.getIntakePosition() >= IntakeConstants.deployIntakeOuterBound-2) {
            intakeS.setPrimaryIntake(-0.5);
            delay.start();
        }

        SmartDashboard.putNumber("Angle Error", intakeS.anglePidController.getPositionError());
        
        intakeS.deployIntake(0.5);

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
