package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class AutoLock extends Command{
    SwerveS swerveS;
    boolean isFinished = false;
    double limelightDeadBand = .1;
    double limelightTx = 0;
    PIDController pidController = new PIDController(0.004, 0, 0);
    ChassisSpeeds speeds;
    public AutoLock(SwerveS swerveS){

        //adds requirements
        this.swerveS = swerveS;
        addRequirements(swerveS);
    }

    @Override
    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){

        //constantly pulls the x value of the limelight, essentially outputs a chassis speed where the only value is a rotational PID loop that ends when deadband is met and sets ChassisSpeeds 
        limelightTx = swerveS.getXError();
        SmartDashboard.putNumber("Auto X Error", limelightTx);
        SmartDashboard.putBoolean("Auto Lock Done", Math.abs(limelightTx)<limelightDeadBand);
        if (Math.abs(limelightTx)<limelightDeadBand){
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
