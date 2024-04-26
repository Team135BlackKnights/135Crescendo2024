package frc.robot.commands.autoCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;

public class FireShooter extends Command {
    private final OutakeS outakeS;
    private final IntakeS intakeS;
    private final double desSpeed;
    private final double time = 0.5;
    private boolean isFinished = false;
    private final Timer timer = new Timer();

    public FireShooter(OutakeS outakeS, IntakeS intakeS, double desSpeed) {
        this.outakeS = outakeS;
        this.intakeS = intakeS;
        this.desSpeed = desSpeed;

        addRequirements(outakeS, intakeS);
    }

    @Override
    public void initialize() {
        //resets timer
        isFinished = false;
        timer.reset();
    }

    @Override
    public void execute() {
        //if the timer hasnt reached the time, essentially uses a pid loop with a feedforward constant (desired velocity/max velocity) to set the motor speed as a percentage
        if (timer.get() >= time) {
            isFinished = true;
        }
        //output velocity and error to smartDashboard
        SmartDashboard.putNumber("Auto Velocity Error", outakeS.shooterPID.getPositionError());
        if (Math.abs(outakeS.shooterPID.getPositionError()) <= 150) {
            //starts timer when within a certain position error, feeds to shoot
            timer.start();
            intakeS.setPrimaryIntake(-0.5);
        }
        outakeS.setRPM(desSpeed);
        if (timer.get() > 0) {
            intakeS.setPrimaryIntake(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        outakeS.setIndividualFlywheelSpeeds(0,0);
        intakeS.setPrimaryIntake(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
