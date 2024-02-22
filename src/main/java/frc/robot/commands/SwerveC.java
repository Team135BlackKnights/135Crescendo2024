
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveS;

public class SwerveC extends Command {
  public ChassisSpeeds chassisSpeeds;
  private final SwerveS swerveS;
  private final boolean fieldOriented = true;
  private final PIDController autoLockController = new PIDController(0.004, 0, 0);
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  
  public SwerveC(SwerveS swerveS) {
    this.swerveS = swerveS;

    // These guys limit acceleration, they aren't the most necessary but it makes movement smoother
    this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAcceleration);
    this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleTurningMaxAcceleration);
    addRequirements(swerveS);
  }
  
  @Override
  public void initialize() {
    
  }
  
  @Override
  public void execute() {
    // Get desired ChassisSpeeds from controller
    double xSpeed = -RobotContainer.driveController.getLeftX();
    double ySpeed = RobotContainer.driveController.getLeftY();
    double turningSpeed = RobotContainer.driveController.getRightX();

    xSpeed = Math.pow(xSpeed, 2) * (xSpeed < 0 ? -1 : 1);
    ySpeed = Math.pow(ySpeed, 2) * (ySpeed < 0 ? -1 : 1);
    turningSpeed = Math.pow(turningSpeed, 2) * (turningSpeed < 0 ? -1 : 1);

    if (SwerveS.autoLock == true) {
      turningSpeed = autoLockController.calculate(swerveS.getXError(), 0.0)*-1;
      SmartDashboard.putNumber("Spin", turningSpeed);
    }
    
    // If the desired ChassisSpeeds are really small (ie from controller drift) make them even smaller so that the robot doesn't move
    xSpeed = Math.abs(xSpeed) > Constants.SwerveConstants.kDeadband ? xSpeed : 0.0001;
    ySpeed = Math.abs(ySpeed) > Constants.SwerveConstants.kDeadband ? ySpeed : 0.0001;
    if (SwerveS.autoLock == true) {
      turningSpeed = Math.abs(turningSpeed) > Constants.SwerveConstants.kAutoDeadband ? turningSpeed : 0.0001;
    } else {
      turningSpeed = Math.abs(turningSpeed) > Constants.SwerveConstants.kDeadband ? turningSpeed : 0.0001;
    }
    
    // Limit the acceleration and convert -1 to 1 from the controller into actual speeds
    xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kMaxSpeedMetersPerSecond; 
    ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kMaxTurningSpeedRadPerSec;
    

    // Convert ChassisSpeeds into the ChassisSpeeds type
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveS.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    
    // Convert ChassisSpeeds into individual module states
    SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Set our module states to our desired module states
    swerveS.setModuleStates(moduleStates);
  }
  
  @Override
  public void end(boolean interrupted) {
    swerveS.stopModules();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
