package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

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
    public PoseEstimate poseEstimate;    

    int periodicUpdateCycle;

    public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-swerve");
    static NetworkTableEntry tx = limelight.getEntry("tx");

    static double xError = tx.getDouble(0.0);

    static Pose2d robotPosition = new Pose2d(0,0, getRotation2d());

    Field2d robotField = new Field2d();

    // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
    ChassisSpeeds m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},robotPosition);
    SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    public static boolean disabled = true;
    public static boolean autoLock = false;
    public static boolean redIsAlliance = true; //used to determine the alliance for LED systems
    static double distance = 0;
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

        limelight.getEntry("pipeline").setNumber(1);

        AutoBuilder.configureHolonomic(
        SwerveS::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
            new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
            Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        SwerveS::getAlliance,
        this // Reference to this subsystem to set requirements
        );

        SmartDashboard.putData("Field", robotField);
    }
    
    public void zeroHeading() {
        gyro.reset();
    }
    
    public static double getHeading() {
        return -1*Math.IEEEremainder(gyro.getAngle() + (getAlliance() ? 180 : 0),360); //modulus
    }
    
    public static Rotation2d getRotation2d() {
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
        periodicUpdateCycle +=1;

        if (getAlliance() && limelight.getEntry("priorityid").getDouble(0.0) != 4.0) {
            limelight.getEntry("priorityid").setNumber(4);
        } else if (!getAlliance() && limelight.getEntry("priorityid").getDouble(0.0) != 7.0) {
            limelight.getEntry("priorityid").setNumber(7);
        }
        

        if(periodicUpdateCycle%5 == 0){
            updatePoseEstimatorWithVisionBotPose();
        }
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
        
        SmartDashboard.putNumber("Position X (getPose)", getPose().getX());
        SmartDashboard.putNumber("Position Y (getPose)", getPose().getY());
        SmartDashboard.putNumber("Robot Heading (getPose)", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("April Tag Distance", getDistanceFromSpeakerInMeters());
        SmartDashboard.putNumber("Odometry Distance", getDistanceFromSpeakerUsingRobotPose());

        SmartDashboard.putNumber("Desired Intake Lower Bound", getDesiredShooterLowerBound());
        SmartDashboard.putNumber("Desired Intake Upper Bound", getDesiredShooterUpperBound());
        SmartDashboard.putNumber("Desired Intake Angle", getDesiredShooterAngle());


        xError = tx.getDouble(0.0);

        m_modulePositions[0] = frontRight.getPosition();
        m_modulePositions[1] = backRight.getPosition();
        m_modulePositions[2] = frontLeft.getPosition();
        m_modulePositions[3] = backLeft.getPosition();

        // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
        m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
        
        robotPosition = poseEstimator.update(getRotation2d(), m_modulePositions);

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

    public static double getXError() {
        return xError;
    }

    public static boolean aprilTagVisible() {
        return xError != 0.0;
    }
    
    public static Pose2d getPose() {
        return robotPosition;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_ChassisSpeeds;
    }
    public static boolean getAlliance(){
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
        poseEstimator.resetPosition(getRotation2d(), m_modulePositions, pose);
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
     public void updatePoseEstimatorWithVisionBotPose() {
        //sanity check, doesn't do anything if unexpected value occurs
        
        //computes latency
        
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-swerve");
        int count = poseEstimate.tagCount;
        Pose2d poseLimelight = poseEstimate.pose;
        double latency = Timer.getFPGATimestamp() - (limelight.getEntry("tl").getDouble(0.0)/1000.0) - (limelight.getEntry("cl").getDouble(0.0)/1000.0);
        Pose2d odomPose = getPose();
        SmartDashboard.putNumber("limelightx", poseLimelight.getX());
        SmartDashboard.putNumber("limelight y", poseLimelight.getY());
        Translation2d transOdom = new Translation2d(odomPose.getX(),odomPose.getY());
        Translation2d transLim = new Translation2d(poseLimelight.getX(),poseLimelight.getY());
        double poseDifference = transOdom.getDistance(transLim);
        //ends if unreasonable result
        if (poseLimelight.getX() == 0){
            return;
        }
        if (count != 0){
            double xyStds;
            double degStds;
            if (count>=2){
                if (periodicUpdateCycle % 10 == 0 && !DriverStation.isAutonomous()) {
                    resetPose(poseLimelight);
                    return;
                } else {
                    xyStds = 0.05;
                    degStds = 6;
                }
            }
            if (poseEstimate.avgTagArea > .8 && poseDifference <.5){
                xyStds = 1.0;
                degStds = 12;
            }
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            } else {
                return;
            }
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            poseEstimator.addVisionMeasurement(poseLimelight, latency);
        }
        else{
            return;
        }
       
        //sanity check, ends if we're getting some unreasonable value (or aprilTags not detected)
        

        }
        /*computes distance from current pose to limelight pose
         * Note: I'm pretty sure pathPlanner (and by extension our odometry)'s coordinate system has its origin at the blue side
         * To avoid issues, we call the blue origin based
        */

        
    

        public static double getDistanceFromSpeakerUsingRobotPose() {
            if (getAlliance()) {
                return robotPosition.getTranslation().getDistance(FieldConstants.redSpeaker);
            } else {
                return robotPosition.getTranslation().getDistance(FieldConstants.blueSpeaker);
            }
        }
        
    






    public static double getDistanceFromSpeakerInMeters(){
         //resets value so it doesn't output last value

        /* if apriltag is detected, uses formula given here https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
        formula is d =(h2-h1)/tan(h2+h1)*/

        if (aprilTagVisible()){
        
            // computing the angle
            double theta = Units.degreesToRadians(LimelightConstants.limeLightAngleOffsetDegrees+limelight.getEntry("ty").getDouble(0.0));
        
            //computes distance
            distance = Units.inchesToMeters(FieldConstants.targetHeightoffFloorInches-LimelightConstants.limelightLensHeightoffFloorInches)/Math.tan(theta);
        }
        return distance;
    }

    public static double getDesiredShooterUpperBound() {
        double upperBoundHeight = 0;
        double upperBoundDistance = 0;
        if (getDistanceFromSpeakerUsingRobotPose() > 5) {
            upperBoundHeight = 0.99*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight;
            upperBoundDistance = 0.95*getDistanceFromSpeakerUsingRobotPose() - FieldConstants.speakerOpeningDepth - DriveConstants.kChassisLength;
        } else {
            upperBoundHeight = 0.99*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight;
            upperBoundDistance = 1.15*getDistanceFromSpeakerUsingRobotPose() - FieldConstants.speakerOpeningDepth - DriveConstants.kChassisLength;
        
        }
        /* if (getDistanceFromSpeakerInMeters() > 6.5) {
            upperBoundHeight = 1.236*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-Units.inchesToMeters(LimelightConstants.limelightLensHeightoffFloorInches);
            upperBoundDistance = 0.764*getDistanceFromSpeakerInMeters() - FieldConstants.speakerOpeningDepth + OutakeConstants.limelightToShooter;
        } else if (getDistanceFromSpeakerInMeters() > 5) {
            upperBoundHeight = 1.2*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-Units.inchesToMeters(LimelightConstants.limelightLensHeightoffFloorInches);
            upperBoundDistance = 0.8*getDistanceFromSpeakerInMeters() - FieldConstants.speakerOpeningDepth + OutakeConstants.limelightToShooter;
        } else {
            upperBoundHeight = 1.12*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-Units.inchesToMeters(LimelightConstants.limelightLensHeightoffFloorInches);
            upperBoundDistance = 0.87*getDistanceFromSpeakerInMeters() - FieldConstants.speakerOpeningDepth + OutakeConstants.limelightToShooter;
        } */
        return Units.radiansToDegrees(Math.atan(upperBoundHeight/upperBoundDistance));
    }

    public static double getDesiredShooterLowerBound() {
        double lowerBoundHeight = 0;
        double lowerBoundDistance = 0;
        if (getDistanceFromSpeakerUsingRobotPose() > 5) {
            lowerBoundHeight = 1.01*FieldConstants.speakerLowerLipHeight + FieldConstants.noteHeight;
            lowerBoundDistance = 0.95*getDistanceFromSpeakerUsingRobotPose() - DriveConstants.kChassisLength;
        } else {
            lowerBoundHeight = 1.01*FieldConstants.speakerLowerLipHeight + FieldConstants.noteHeight;
            lowerBoundDistance = 1.15*getDistanceFromSpeakerUsingRobotPose() - DriveConstants.kChassisLength;
        }
        /* if (getDistanceFromSpeakerInMeters() > 6) {
            lowerBoundHeight = 1.242*FieldConstants.speakerLowerLipHeight+FieldConstants.noteHeight-Units.inchesToMeters(LimelightConstants.limelightLensHeightoffFloorInches);
            lowerBoundDistance = 0.758*getDistanceFromSpeakerInMeters() + OutakeConstants.limelightToShooter;
        } else if (getDistanceFromSpeakerInMeters() > 4) {
            lowerBoundHeight = 1.2*FieldConstants.speakerLowerLipHeight+FieldConstants.noteHeight-Units.inchesToMeters(LimelightConstants.limelightLensHeightoffFloorInches);
            lowerBoundDistance = 0.8*getDistanceFromSpeakerInMeters() + OutakeConstants.limelightToShooter;
        } else {
            lowerBoundHeight = 1.12*FieldConstants.speakerLowerLipHeight+FieldConstants.noteHeight-Units.inchesToMeters(LimelightConstants.limelightLensHeightoffFloorInches);
            lowerBoundDistance = 0.87*getDistanceFromSpeakerInMeters() + OutakeConstants.limelightToShooter;
        } */
        return Units.radiansToDegrees(Math.atan(lowerBoundHeight/lowerBoundDistance));
    }

    public static double getDesiredShooterAngle() {
        return (getDesiredShooterUpperBound() + getDesiredShooterLowerBound())/2;
    }

    public InstantCommand toggleAutoLockCommand() {
        return new InstantCommand(this::toggleAutoLock, this);
    }

    public void setChassisSpeeds(ChassisSpeeds speed) {
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
