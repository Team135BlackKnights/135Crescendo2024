package frc.robot.commands.autoCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    
    private Translation2d targetNoteLocation = null;
    private Pose2d currentPose;
    private ChassisSpeeds speeds;
    double ty;
    Timer timer = new Timer();
    Timer delayTimer = new Timer();
    double desiredHeading,currentHeading, error;
    /*Call this in all cases but simulation autonomous*/
    public AutonIntake(IntakeS intakeS, SwerveS swerveS) {
        this.intakeS = intakeS;
        this.swerveS = swerveS;
        addRequirements(intakeS);
        this.targetNoteLocation = intakeS.getClosestNote();
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
        this.targetNoteLocation = intakeS.getClosestNote();
        intakeS.deployIntake(intakeS.outsideBotState());
    }

    @Override
    public void execute() {
        double tx, ty;
        boolean tv;
        if (Robot.isSimulation()){
            //Computing distance
            currentPose = SwerveS.getPose();
            tx = targetNoteLocation.getX() - currentPose.getX();// -0.307975;
            ty = targetNoteLocation.getY() - currentPose.getY();
            tv = true;
            // Calculate the angle to aim towards the target
            double angleToTarget = Math.atan2(ty, tx); // Calculate angle in radians
            // Convert angle to degrees if necessary
            angleToTarget = Math.toDegrees(angleToTarget);
            SmartDashboard.putNumber("ANGLE", angleToTarget);

            // Adjust the robot's orientation towards the target
            // Example: Set the desired heading for your motion control system
            desiredHeading = angleToTarget;
            currentHeading = SwerveS.getHeading();
            error = currentHeading-desiredHeading;
        }else{
            IntakeS.detected = IntakeS.colorSensorV3.getColor();
            IntakeS.colorMatchResult = IntakeS.colorMatch.matchClosestColor(IntakeS.detected);

            //when done, set timer.start().. and delayTimer.stop();
            tx = LimelightHelpers.getTX(Constants.LimelightConstants.limelightName);
            tv = LimelightHelpers.getTV(Constants.LimelightConstants.limelightName);
            ty = LimelightHelpers.getTY(Constants.LimelightConstants.limelightName);
            error = CameraS.calculateAngleFromTX(tx,ty);
            SmartDashboard.putNumber("ANGLE ERROR", error);
            
            }
           // SmartDashboard.putBoolean("Note Loaded?", IntakeS.noteIsLoaded());
           if (Robot.isReal()){
            if (ty >18.5){
                close =true;
            }
           } else{
            if (Math.abs(ty) < Units.inchesToMeters(4) && Math.abs(tx) < Units.inchesToMeters(4)){
                close = true;
            }
           }
           
            if (tv == false && loaded==false && close == false) {
                // We don't see the target, seek for the target by spinning in place at a safe speed.
                speeds = new ChassisSpeeds(0,0,0.1*Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
            } else if (loaded==false && close == false) {
                double moveSpeed = Constants.IntakeConstants.macroMoveSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
                speeds = new ChassisSpeeds(moveSpeed,0,IntakeS.autoIntakeController.calculate(error,0)*Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
            }else{
                speeds = new ChassisSpeeds(0,0,0);
            }
            swerveS.setChassisSpeeds(speeds);
            intakeS.setPrimaryIntake(-0.5);
            //runs intake if note is not loaded
            if (Robot.isReal()){
                if (IntakeS.noteIsLoaded() || delayTimer.get() > 2) {
                    delayTimer.stop();
                    isFinished = true;
                }
            }else{
                if (close || delayTimer.get() > 2){
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
        System.out.println("GOT NOTE");
        close = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
