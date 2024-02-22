package frc.robot.commands.autoCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.OutakeS;

public class FireShooter extends Command {
    private final OutakeS outakeS;
    private final IntakeS intakeS;
    private final double desSpeed;
    private final double time = 2;
    private boolean isFinished = false;
    private final PIDController shooterPID = new PIDController(0.0002, 0, 0);
    private final  double feedforward;
    private final Timer timer = new Timer();

    public FireShooter(OutakeS outakeS, IntakeS intakeS, double desSpeed) {
        this.outakeS = outakeS;
        this.intakeS = intakeS;
        this.desSpeed = desSpeed;
        this.feedforward = desSpeed/Constants.OutakeConstants.flywheelMaxRPM;

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
        double output = feedforward + shooterPID.calculate(outakeS.getAverageFlywheelSpeed(), desSpeed);
        //output velocity and error to smartDashboard
        SmartDashboard.putNumber("Auto Shooter Output", output);
        SmartDashboard.putNumber("Auto Velocity Error", shooterPID.getPositionError());
        if (Math.abs(shooterPID.getPositionError()) <= 100) {
            //starts timer when within a certain position error, feeds to shoot
            timer.start();
            intakeS.setPrimaryIntake(-0.5);
        }
        outakeS.setFiringSpeed(output);
        if (timer.get() > 0) {
            intakeS.setPrimaryIntake(-0.5);
        }
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
