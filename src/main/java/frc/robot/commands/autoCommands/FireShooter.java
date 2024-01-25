package frc.robot.commands.autoCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OutakeS;

public class FireShooter extends Command {
    private final OutakeS outakeS;
    private final double desSpeed;
    private final double time;
    private boolean isFinished = false;
    private final PIDController shooterPID = new PIDController(0, 0, 0);
    private final Timer timer = new Timer();

    public FireShooter(OutakeS outakeS, double desSpeed, double time) {
        this.outakeS = outakeS;
        this.desSpeed = desSpeed;
        this.time = time;

        addRequirements(outakeS);
    }

    @Override
    public void initialize() {
        isFinished = false;
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.get() >= time) {
            isFinished = true;
        }
        double output = shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), desSpeed);
        if (Math.abs(shooterPID.getVelocityError()) <= 0.5) {
            timer.start();
        }
        outakeS.setFiringSpeed(output);
    }

    @Override
    public void end(boolean interrupted) {
        outakeS.setFiringSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
