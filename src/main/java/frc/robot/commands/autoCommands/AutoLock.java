package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;

public class AutoLock extends Command{
    SwerveS swerveS;
    boolean isFinished = false;
    double limelightDeadBand = .01;
    double limelightTx = 0;
    PIDController pidController = new PIDController(5, 0, 0);
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
        limelightTx = swerveS.getXError();
        if (Math.abs(limelightTx)<limelightDeadBand){
            isFinished = true;
        }
        speeds = new ChassisSpeeds(0,0,pidController.calculate(limelightTx,0));
        swerveS.setChassisSpeeds(speeds);
    }

    @Override 
    public void end(boolean interrupted){
        speeds = new ChassisSpeeds();
        swerveS.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
