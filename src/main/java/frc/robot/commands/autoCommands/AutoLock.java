package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class AutoLock extends Command{

    SwerveS swerveS;
    boolean isFinished = false;
    double limelightDeadBand = .05, limelightTx = 0, maxTimeTargetting = 1.5;
    PIDController pidController = new PIDController(0.004, 0, 0);
    ChassisSpeeds speeds;

    private Timer timer = new Timer();

    public AutoLock(SwerveS swerveS){

        //adds requirements
        this.swerveS = swerveS;
        addRequirements(swerveS);
    }

    @Override
    public void initialize(){
        isFinished = false;
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute(){
        if (timer.get() >= maxTimeTargetting) {
            isFinished = true;
        }

        if (!swerveS.aprilTagVisible()) {
            timer.start();
        }

        //constantly pulls the x value of the limelight, essentially outputs a chassis speed where the only value is a rotational PID loop that ends when deadband is met and sets ChassisSpeeds 
        limelightTx = swerveS.getXError();
        SmartDashboard.putNumber("Auto X Error", limelightTx);
        SmartDashboard.putBoolean("Auto Lock Done", Math.abs(limelightTx) < limelightDeadBand);
        if (Math.abs(limelightTx) < limelightDeadBand && swerveS.aprilTagVisible()){
            isFinished = true;
        }
        speeds = new ChassisSpeeds(0,0,pidController.calculate(limelightTx,0)*-Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
        swerveS.setChassisSpeeds(speeds);
    }

    @Override 
    public void end(boolean interrupted){

        //when its done, set the chassisSpeeds to 0 and output it to swerve chassis (stop drivetrain from moving)
        speeds = new ChassisSpeeds(0,0,0);
        swerveS.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
