package frc.robot.commands.autoCommands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinMotor extends Command {
    private CANSparkMax motor;
    private double time;
    private Timer timer = new Timer();
    private boolean isFinished = false;
    public SpinMotor(CANSparkMax motor, double time){
        this.motor = motor;
        this.time = time;
    }

    @Override
    public void initialize() {
        isFinished = false;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        motor.set(0.3);
        if (timer.get() > time) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
