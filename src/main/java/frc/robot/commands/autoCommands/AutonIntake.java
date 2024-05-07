package frc.robot.commands.autoCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
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
    
    private Translation2d targetNoteLocation = null;
    private Pose2d currentPose;
    private ChassisSpeeds speeds;
    double ty;
    Timer timer = new Timer();
    Timer delayTimer = new Timer();
    /*Call this in all cases but simulation autonomous*/
    public AutonIntake(IntakeS intakeS, SwerveS swerveS) {
        this.intakeS = intakeS;
        this.swerveS = swerveS;
        addRequirements(intakeS);
        if (DriverStation.isTeleop()){
            this.targetNoteLocation = intakeS.getClosestNote();
        }
    }
    /*Call this for simulation autonomous only */
    public AutonIntake(IntakeS intakeS, SwerveS swerveS, Translation2d fieldNotePose){
        this.intakeS = intakeS;
        this.swerveS = swerveS;
        this.targetNoteLocation = fieldNotePose;
        
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
        double tx, ty;
        boolean tv;
        if (Robot.isSimulation()){
            //Computing distance
            currentPose = SwerveS.getPose();
            tx = currentPose.getX()-targetNoteLocation.getX();
            ty = currentPose.getY()-targetNoteLocation.getY();
            tv = true;
            
        }else{
            IntakeS.detected = IntakeS.colorSensorV3.getColor();
            IntakeS.colorMatchResult = IntakeS.colorMatch.matchClosestColor(IntakeS.detected);

            //when done, set timer.start().. and delayTimer.stop();
            intakeS.deployIntake(1);
            tx = LimelightHelpers.getTX(Constants.LimelightConstants.limelightName);
            tv = LimelightHelpers.getTV(Constants.LimelightConstants.limelightName);
            ty = LimelightHelpers.getTY(Constants.LimelightConstants.limelightName);
            SmartDashboard.putNumber("TY", ty);
            
            }
            SmartDashboard.putBoolean("Note Loaded?", IntakeS.noteIsLoaded());
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
