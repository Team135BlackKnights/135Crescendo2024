package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    private AHRS gyro = new AHRS(Port.kUSB1);
    NetworkTableEntry pipeline;
    

    public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-swerve");
    NetworkTableEntry tx = limelight.getEntry("tx");
    static NetworkTableEntry tv = limelight.getEntry("tv");

    double xError = tx.getDouble(0.0);
    static double aprilTagVisible = tv.getDouble(0.0);

    Pose2d robotPosition = new Pose2d(0,0, getRotation2d());

    Field2d robotField = new Field2d();

    // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
    ChassisSpeeds m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},robotPosition);
    SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    public static boolean disabled = true;
    public static boolean autoLock = false;
    public static boolean redIsAlliance = true; //used to determine the alliance for LED systems
    public SwerveS() {
        // Waits for the RIO to finishing booting
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
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
            new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
            new PIDConstants(2, 0.0, 0.0), // Rotation PID constants
            Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this::getAlliance,
        this // Reference to this subsystem to set requirements
        );

        SmartDashboard.putData("Field", robotField);
    }
    
    public void zeroHeading() {
        gyro.reset();
    }
    
    public double getHeading() {
        return -1*Math.IEEEremainder(gyro.getAngle() + (getAlliance() ? 180 : 0),360); //modulus
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
        redIsAlliance = getAlliance();
        disabled = DriverStation.isDisabled();
        //puts values to smartDashboard
        SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        SmartDashboard.putNumber("FrontLeft Abs Encoder", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FrontRight Abs Encoder", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BackLeft Abs Encoder", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BackRight Abs Encoder", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("xError", xError);
        SmartDashboard.putBoolean("Auto Lock", autoLock);
        SmartDashboard.putBoolean("Red is Alliance", getAlliance());
        
        // SmartDashboard.putNumber("BackRight Position (SwerveModulePosition)", backRight.getPosition().distanceMeters);
        // SmartDashboard.putNumber("FrontRight Position (SwerveModulePosition)", frontRight.getPosition().distanceMeters);
        // SmartDashboard.putNumber("BackLeft Position (SwerveModulePosition)", backLeft.getPosition().distanceMeters);
        // SmartDashboard.putNumber("FrontLeft Position (SwerveModulePosition)", frontLeft.getPosition().distanceMeters);
        // SmartDashboard.putNumber("BackRight Angle (SwerveModulePosition)", backRight.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("FrontRight Angle (SwerveModulePosition)", frontRight.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("BackLeft Angle (SwerveModulePosition)", backLeft.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("FrontLeft Angle (SwerveModulePosition)", frontLeft.getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Position X (getPose)", getPose().getX());
        SmartDashboard.putNumber("Position Y (getPose)", getPose().getY());
        SmartDashboard.putNumber("Robot Heading (getPose)", getPose().getRotation().getDegrees());

        xError = tx.getDouble(0.0);
        aprilTagVisible = tv.getDouble(0.0);

        m_modulePositions[0] = frontRight.getPosition();
        m_modulePositions[1] = backRight.getPosition();
        m_modulePositions[2] = frontLeft.getPosition();
        m_modulePositions[3] = backLeft.getPosition();

        // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
        m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});

        robotPosition = odometry.update(getRotation2d(), m_modulePositions);

        robotField.setRobotPose(getPose());


        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            robotField.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            robotField.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            robotField.getObject("path").setPoses(poses);
        });

    }

    public double getXError() {
        return xError;
    }

    public static boolean aprilTagVisible() {
        return aprilTagVisible == 1;
    }
    
    public Pose2d getPose() {
        Pose2d actualPosition = new Pose2d(-robotPosition.getY(), robotPosition.getX(), getRotation2d());
        return actualPosition;
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
        pose = new Pose2d(pose.getY(), -pose.getX(), getRotation2d());
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
        speed = new ChassisSpeeds(speed.vyMetersPerSecond, -speed.vxMetersPerSecond, speed.omegaRadiansPerSecond);
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
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
