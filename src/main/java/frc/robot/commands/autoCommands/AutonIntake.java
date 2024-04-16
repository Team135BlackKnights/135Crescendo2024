package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.SwerveS;

public class AutonIntake extends Command {
    private final IntakeS intakeS;
    private final SwerveS swerveS;
    private boolean isFinished = false;
    private boolean loaded = false;
    public static boolean allClear = false;
    private PIDController notePIDController;
    private ChassisSpeeds speeds;
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
        LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.limelightName, 2);
        notePIDController = new PIDController(Constants.LimelightConstants.PIDConstants.P,Constants.LimelightConstants.PIDConstants.I,Constants.LimelightConstants.PIDConstants.D);
        delayTimer.reset();
        delayTimer.start();
        allClear = false;
    }

    @Override
    public void execute() {
        IntakeS.detected = IntakeS.colorSensorV3.getColor();
        IntakeS.colorMatchResult = IntakeS.colorMatch.matchClosestColor(IntakeS.detected);
        SmartDashboard.putBoolean("Note Loaded?", IntakeS.noteIsLoaded());
        //when done, set timer.start().. and delayTimer.stop();
        intakeS.deployIntake(1);

        double tx = SwerveS.getXError();
        boolean tv = LimelightHelpers.getTV(Constants.LimelightConstants.limelightName);
        double ta = LimelightHelpers.getTA(Constants.LimelightConstants.limelightName);
        if (tv == false && loaded==false) {
        // We don't see the target, seek for the target by spinning in place at a safe speed.
        speeds = new ChassisSpeeds(0,0,0.2*Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
        } else if (loaded==false) {
            double moveSpeed = notePIDController.calculate(ta,Constants.LimelightConstants.PIDConstants.intakeTaOffset);
            speeds = new ChassisSpeeds(moveSpeed,0,swerveS.autoLockController.calculate(tx,0)*Constants.DriveConstants.kMaxTurningSpeedRadPerSec);
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
        intakeS.setPrimaryIntake(0);
        timer.stop();
        timer.reset();
        delayTimer.stop();
        delayTimer.reset();
        speeds = new ChassisSpeeds(0,0,0);
        swerveS.setChassisSpeeds(speeds);
        allClear = true;
        intakeS.pullBackNote(); 
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
