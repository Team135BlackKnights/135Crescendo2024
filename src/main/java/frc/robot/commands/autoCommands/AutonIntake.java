package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CameraS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.SwerveS;

public class AutonIntake extends Command {
    private final IntakeS intakeS;
    private final SwerveS swerveS;
    private boolean isFinished = false;
    private boolean loaded = false;
    public static boolean allClear = false;
    public static boolean takeOver = false;
    private static boolean close = false;
    private ChassisSpeeds speeds;
    double ty;
    Timer timer = new Timer();
    Timer delayTimer = new Timer();
    public AutonIntake(IntakeS intakeS, SwerveS swerveS) {
        this.intakeS = intakeS;
        this.swerveS = swerveS;
        addRequirements(intakeS);
    }

    @Override
    public void initialize() {
        isFinished = false;
        LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.limelightName, 1); //TODO: Make this base Pipe, cuz error swapping back and forth
        delayTimer.reset();
        delayTimer.start();
        allClear = false;
        close = false;
        takeOver = true;
        ty = 0;
    }

    @Override
    public void execute() {
        if (Robot.isSimulation()){
            if (delayTimer.get() > 1){
                delayTimer.stop();
                allClear = true;
                isFinished = true;
            }
        }else{
            IntakeS.detected = IntakeS.colorSensorV3.getColor();
            IntakeS.colorMatchResult = IntakeS.colorMatch.matchClosestColor(IntakeS.detected);
            SmartDashboard.putBoolean("Note Loaded?", IntakeS.noteIsLoaded());
            //when done, set timer.start().. and delayTimer.stop();
            intakeS.deployIntake(1);
            double tx = LimelightHelpers.getTX(Constants.LimelightConstants.limelightName);
            boolean tv = LimelightHelpers.getTV(Constants.LimelightConstants.limelightName);
            ty = LimelightHelpers.getTY(Constants.LimelightConstants.limelightName);
            SmartDashboard.putNumber("TY", ty);
            if (ty >18.5){
                close =true;
            }
                if (tv == false && loaded==false && close == false) {
            // We don't see the target, seek for the target by spinning in place at a safe speed.
            speeds = new ChassisSpeeds(0,0,0.1*Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
            } else if (loaded==false && close == false) {
                double moveSpeed = Constants.IntakeConstants.macroMoveSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
                speeds = new ChassisSpeeds(moveSpeed,0,IntakeS.autoIntakeController.calculate(tx,0)*Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
            }else{
                speeds = new ChassisSpeeds(0,0,0);
            }
            swerveS.setChassisSpeeds(speeds);
            intakeS.setPrimaryIntake(-0.5);
            //runs intake if note is not loaded
            if (IntakeS.noteIsLoaded() || delayTimer.get() > 2) {
                delayTimer.stop();
                isFinished = true;
            }
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        takeOver = false;
        intakeS.setPrimaryIntake(0);
        timer.stop();
        timer.reset();
        delayTimer.stop();
        delayTimer.reset();
        speeds = new ChassisSpeeds(0,0,0);
        swerveS.setChassisSpeeds(speeds);
        intakeS.pullBackNote(); 
        System.out.println("DONE INTAKING");
        close = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
