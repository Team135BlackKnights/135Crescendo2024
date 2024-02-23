package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class AutoLock extends Command{

    SwerveS swerveS;
    boolean isFinished = false;
    double limelightDeadBand = 1, limelightTx = 0, maxTimeTargetting = 1.5;
    PIDController pidController = new PIDController(0.004, 0, 0);
    ChassisSpeeds speeds;
    public AutoLock(SwerveS swerveS){
        this.swerveS = swerveS;
        addRequirements(swerveS);
    }

    @Override
    public void initialize(){
        isFinished = false;
    }

    @Override
    public void execute(){
        //takes x value of limelight as the distance nthat needs to be rotated, runs a chassisSpeedscommand to rotate until within an acceptable deadband
        limelightTx = swerveS.getXError();
        if (Math.abs(limelightTx)<limelightDeadBand){
            isFinished = true;
        }
        speeds = new ChassisSpeeds(0,0,pidController.calculate(limelightTx,0)*-Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
        swerveS.setChassisSpeeds(speeds);
    }

    @Override 
    public void end(boolean interrupted){
        speeds = new ChassisSpeeds(0,0,0);
        swerveS.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
