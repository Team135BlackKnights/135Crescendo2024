
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DataLog;
import frc.robot.subsystems.SwerveS;

public class SwerveC extends Command {
  public ChassisSpeeds chassisSpeeds;
  private final SwerveS swerveS;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private int arrayIndex = 0;
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
    double xSpeed = -RobotContainer.driveController.getLeftY();
    double ySpeed = -RobotContainer.driveController.getLeftX();
    double turningSpeed = -RobotContainer.driveController.getRightX();

    xSpeed = Math.pow(xSpeed, 2) * (xSpeed < 0 ? -1 : 1);
    ySpeed = Math.pow(ySpeed, 2) * (ySpeed < 0 ? -1 : 1);
    //turningSpeed = Math.pow(turningSpeed, 2) * (turningSpeed < 0 ? -1 : 1);

    if (RobotContainer.driveController.getRightBumper() == true) {
      xSpeed /=2;
      ySpeed /=2;
    }

    if (SwerveS.autoLock == true && SwerveS.aprilTagVisible() == true) {
      turningSpeed = swerveS.autoLockController.calculate(SwerveS.getXError(), 0.0);
      //SmartDashboard.putNumber("Spin", turningSpeed);
    }

    // If the desired ChassisSpeeds are really small (ie from controller drift) make them even smaller so that the robot doesn't move
    xSpeed = Math.abs(xSpeed) > Constants.SwerveConstants.kDeadband ? xSpeed : 0.0001;
    ySpeed = Math.abs(ySpeed) > Constants.SwerveConstants.kDeadband ? ySpeed : 0.0001;
    if (SwerveS.autoLock == true && SwerveS.aprilTagVisible() == true) {
      turningSpeed = Math.abs(turningSpeed) > Constants.SwerveConstants.kAutoDeadband ? turningSpeed : 0.0001;
    } else {
      turningSpeed = Math.abs(turningSpeed) > Constants.SwerveConstants.kDeadband ? turningSpeed : 0.0001;
    }
    
    // Limit the acceleration and convert -1 to 1 from the controller into actual speeds
    xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kMaxSpeedMetersPerSecond; 
    ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kMaxTurningSpeedRadPerSec;
    

    if (SwerveS.getAlliance()) {
      xSpeed *= -1;
      ySpeed *= -1;
      turningSpeed *= 1;
    }

    // Convert ChassisSpeeds into the ChassisSpeeds type
    if (SwerveS.fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, SwerveS.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
    
    // Convert ChassisSpeeds into individual module states
    SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Set our module states to our desired module states
    swerveS.setModuleStates(moduleStates);
  }
  /*Use this link to compute the regression model:https://planetcalc.com/5992/#google_vignette 
    Each of the files has an x and y output so put those in the respective lists, or use a ti-84 stats bar*/
  public void printData() {
    System.out.println("Distance (X)                              Angle (Y)");
      for (var i = 0; i < arrayIndex; i++){
        String output = Double.toString(DataLog.variableAngleLog[0][i]).concat("    "+Double.toString(DataLog.variableAngleLog[1][i]));
        System.out.println(output);
        
      }
      System.out.println("Distance (X)");
      for (var i = 0; i < arrayIndex; i++){
        String output = Double.toString(DataLog.variableAngleLog[0][i]) +" ";
        System.out.println(output);
      }
      System.out.println("Angle (Y)");
      for (var i = 0; i < arrayIndex; i++){
        String output = Double.toString(DataLog.variableAngleLog[1][i]) +" ";
        System.out.println(output);
        
      }

      DataLog.variableAngleLog = new double[2][20];
      arrayIndex = 0;
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
