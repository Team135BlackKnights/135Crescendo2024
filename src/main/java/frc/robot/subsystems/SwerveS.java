package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class SwerveS extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.DriveConstants.kFrontLeftDrivePort, 
        Constants.DriveConstants.kFrontLeftTurningPort, 
        Constants.DriveConstants.kFrontLeftDriveReversed, 
        Constants.DriveConstants.kFrontLeftTurningReversed, 
        Constants.DriveConstants.kFrontLeftAbsEncoderPort, 
        Constants.DriveConstants.kFrontLeftAbsEncoderOffsetRad, 
        Constants.DriveConstants.kFrontLeftAbsEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.DriveConstants.kFrontRightDrivePort, 
        Constants.DriveConstants.kFrontRightTurningPort, 
        Constants.DriveConstants.kFrontRightDriveReversed, 
        Constants.DriveConstants.kFrontRightTurningReversed, 
        Constants.DriveConstants.kFrontRightAbsEncoderPort, 
        Constants.DriveConstants.kFrontRightAbsEncoderOffsetRad, 
        Constants.DriveConstants.kFrontRightAbsEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        Constants.DriveConstants.kBackLeftDrivePort, 
        Constants.DriveConstants.kBackLeftTurningPort, 
        Constants.DriveConstants.kBackLeftDriveReversed, 
        Constants.DriveConstants.kBackLeftTurningReversed, 
        Constants.DriveConstants.kBackLeftAbsEncoderPort, 
        Constants.DriveConstants.kBackLeftAbsEncoderOffsetRad, 
        Constants.DriveConstants.kBackLeftAbsEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        Constants.DriveConstants.kBackRightDrivePort, 
        Constants.DriveConstants.kBackRightTurningPort, 
        Constants.DriveConstants.kBackRightDriveReversed, 
        Constants.DriveConstants.kBackRightTurningReversed, 
        Constants.DriveConstants.kBackRightAbsEncoderPort, 
        Constants.DriveConstants.kBackRightAbsEncoderOffsetRad, 
        Constants.DriveConstants.kBackRightDriveReversed);

    private static AHRS gyro = new AHRS(Port.kUSB1);
    NetworkTableEntry pipeline;
    

    public NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-swerve");
    NetworkTableEntry tx = limelight.getEntry("tx");
    double xError = tx.getDouble(0.0);

    Pose2d robotPosition = new Pose2d(0,0, getRotation2d());

    // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
    ChassisSpeeds m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},robotPosition);
    SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    public boolean lockedOntoAprilTag;
    public boolean autoLock = false;
    public static boolean redIsAlliance = true; //used to determine the alliance for LED systems
    public SwerveS() {
        // Waits for the RIO to finishing booting
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                redIsAlliance = getAlliance();
                frontLeft.resetEncoders();
                frontRight.resetEncoders();
                backLeft.resetEncoders();
                backRight.resetEncoders();                
            } catch (Exception e) {
            }
        }).start();

        AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), this::getAlliance
       ,
        this // Reference to this subsystem to set requirements
        );

    }
    
    public void zeroHeading() {
        gyro.reset();
    }
    
    public double getHeading() {
        return -1*Math.IEEEremainder(gyro.getAngle(),360); //modulus
    }

    public static double getTilt() {
        return Math.IEEEremainder(gyro.getRoll(),360); //either getRoll() or getPitch()
    }
    
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    public void switchLimeLightPath() { 
        //basically has 2 paths: target red and blue speakers and targets everything else 
        pipeline = limelight.getEntry("pipeline");
        if ((int)pipeline.getNumber(0)==0){
            pipeline.setNumber(1);
        }else{
            pipeline.setNumber(0);
        }
    }
    public InstantCommand switchLimeLightPathCommand(){
        return new InstantCommand(this::switchLimeLightPath,this);
    }

    
    @Override
    public void periodic() {
        //puts values to smartDashboard
        SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        SmartDashboard.putNumber("FrontLeft Abs Encoder", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FrontRight Abs Encoder", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BackLeft Abs Encoder", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BackRight Abs Encoder", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("xError", xError);
        SmartDashboard.putBoolean("Auto Lock", autoLock);
        SmartDashboard.putBoolean("Red is Alliance", redIsAlliance);
        
        // SmartDashboard.putNumber("BackRight Position (SwerveModulePosition)", backRight.getPosition().distanceMeters);
        // SmartDashboard.putNumber("FrontRight Position (SwerveModulePosition)", frontRight.getPosition().distanceMeters);
        // SmartDashboard.putNumber("BackLeft Position (SwerveModulePosition)", backLeft.getPosition().distanceMeters);
        // SmartDashboard.putNumber("FrontLeft Position (SwerveModulePosition)", frontLeft.getPosition().distanceMeters);
        // SmartDashboard.putNumber("BackRight Angle (SwerveModulePosition)", backRight.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("FrontRight Angle (SwerveModulePosition)", frontRight.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("BackLeft Angle (SwerveModulePosition)", backLeft.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("FrontLeft Angle (SwerveModulePosition)", frontLeft.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Position X (getPose)", getPose().getX());
        // SmartDashboard.putNumber("Position Y (getPose)", getPose().getY());
        // SmartDashboard.putNumber("Robot Heading (getPose)", getPose().getRotation().getDegrees());

        xError = tx.getDouble(0.0);
        lockedOntoAprilTag = (limelight.getEntry("tv").getDouble(0.0)==1);
        m_modulePositions[0] = frontLeft.getPosition();
        m_modulePositions[1] = frontRight.getPosition();
        m_modulePositions[2] = backLeft.getPosition();
        m_modulePositions[3] = backRight.getPosition();

        // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
        m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});

        robotPosition = odometry.update(getRotation2d(), m_modulePositions);
    }

    public double getXError() {
        return xError;
    }

    






    public double getDistanceFromSpeakerInMeters(){

        double distance = 0; //resets value so it doesn't output last value

        /* if apriltag is detected, uses formula given here https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
        formula is d =(h2-h1)/tan(h2+h1)*/

        if (lockedOntoAprilTag){
        
            // computing the angle
            double theta = Units.degreesToRadians(LimelightConstants.limeLightAngleOffsetDegrees+limelight.getEntry("ty").getDouble(0.0));
        
            //computes distance
            distance = Units.inchesToMeters(LimelightConstants.targetHeightoffFloorInches-LimelightConstants.limelightLensHeightoffFloorInches)/Math.tan(theta);}

        else{
            
            //if robot does not have a lock onto april tag try to approximate distance from speaker with robot odometry
            
            //pulls robot pose and converts it to a translation
            Pose2d pose = getPose();
            Translation2d translation = new Translation2d(pose.getX(),pose.getY());
            //compares it with the translation2d of the speaker (determined through pathPlanner)
             distance = translation.getDistance(new Translation2d(0,5.55));
        }


        
        return distance;
    }
    public Pose2d getPose() {
        return robotPosition;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_ChassisSpeeds;
    }
    public boolean getAlliance(){
        // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
    }


    public void resetPose(Pose2d pose) {
        // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, pose);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void toggleAutoLock() {
        autoLock = !autoLock;
    }

    public InstantCommand toggleAutoLockCommand() {
        return new InstantCommand(this::toggleAutoLock, this);
    }

    public void setChassisSpeeds(ChassisSpeeds speed) {
        speed.omegaRadiansPerSecond = speed.omegaRadiansPerSecond * -1;
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        setModuleStates(moduleStates);
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Proportionally lowers wheel speeds until they are under the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        
        // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
