package frc.robot.commands.autoCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;

public class FireShooter extends Command {
    private final OutakeS outakeS;
    private final IntakeS intakeS;
    private final double desSpeed;
    private final double time = 2;
    private boolean isFinished = false;
    private final PIDController shooterPID = new PIDController(0.0005, 0.0002, 0);
    private final Timer timer = new Timer();

    public FireShooter(OutakeS outakeS, IntakeS intakeS, double desSpeed) {
        this.outakeS = outakeS;
        this.intakeS = intakeS;
        this.desSpeed = desSpeed;

        addRequirements(outakeS);
    }

    @Override
    public void initialize() {
        isFinished = false;
        timer.reset();
    }

    @Override
    public void execute() {
        //if (timer.get() >= time) {
        //    isFinished = true;
        //}
        double output = shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), desSpeed);
        SmartDashboard.putNumber("Auto Shooter Output", output);
        SmartDashboard.putNumber("Auto Velocity Error", shooterPID.getPositionError());
        if (Math.abs(shooterPID.getPositionError()) <= 100) {
            timer.start();
            intakeS.setPrimaryIntake(-1);
        }
        outakeS.setFiringSpeed(output);
    }

    @Override
    public void end(boolean interrupted) {
        outakeS.setFiringSpeed(0);
        intakeS.setPrimaryIntake(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
