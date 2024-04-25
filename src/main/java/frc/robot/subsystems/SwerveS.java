package frc.robot.subsystems;

import frc.robot.DataHandler;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import static edu.wpi.first.units.Units.Volts;


public class SwerveS extends SubsystemBase {
    
    private final static SwerveModule frontLeft = new SwerveModule(
        Constants.DriveConstants.kFrontLeftDrivePort, 
        Constants.DriveConstants.kFrontLeftTurningPort, 
        Constants.DriveConstants.kFrontLeftDriveReversed, 
        Constants.DriveConstants.kFrontLeftTurningReversed, 
        Constants.DriveConstants.kFrontLeftAbsEncoderPort, 
        Constants.DriveConstants.kFrontLeftAbsEncoderOffsetRad, 
        Constants.DriveConstants.kFrontLeftAbsEncoderReversed);

    private final static SwerveModule frontRight = new SwerveModule(
        Constants.DriveConstants.kFrontRightDrivePort, 
        Constants.DriveConstants.kFrontRightTurningPort, 
        Constants.DriveConstants.kFrontRightDriveReversed, 
        Constants.DriveConstants.kFrontRightTurningReversed, 
        Constants.DriveConstants.kFrontRightAbsEncoderPort, 
        Constants.DriveConstants.kFrontRightAbsEncoderOffsetRad, 
        Constants.DriveConstants.kFrontRightAbsEncoderReversed);

    private final static SwerveModule backLeft = new SwerveModule(
        Constants.DriveConstants.kBackLeftDrivePort, 
        Constants.DriveConstants.kBackLeftTurningPort, 
        Constants.DriveConstants.kBackLeftDriveReversed, 
        Constants.DriveConstants.kBackLeftTurningReversed, 
        Constants.DriveConstants.kBackLeftAbsEncoderPort, 
        Constants.DriveConstants.kBackLeftAbsEncoderOffsetRad, 
        Constants.DriveConstants.kBackLeftAbsEncoderReversed);

    private final static SwerveModule backRight = new SwerveModule(
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
    public static boolean fieldOriented = true;
    int periodicUpdateCycle;

    public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.limelightName);
    static NetworkTableEntry tx = limelight.getEntry("tx");
    static double xError = tx.getDouble(0.0);

    static Pose2d robotPosition = new Pose2d(0,0, getRotation2d());

    Field2d robotField = new Field2d();
    static double zAccel = 0;
    // LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
    ChassisSpeeds m_ChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
    static SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},robotPosition);
    SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    public static boolean autoLock = false;

    public static boolean redIsAlliance = true;

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                frontLeft.setTurningTest(volts.in(Volts));
                frontRight.setTurningTest(volts.in(Volts));
                backLeft.setTurningTest(volts.in(Volts));
                backRight.setTurningTest(volts.in(Volts));
              },
          null // No log consumer, since data is recorded by URCL
    , this
        )
      );
    

    public PIDController autoLockController = new PIDController(0.0044, 0.00135, 0.00001);

    //so that the navXDisconnect command doesn't start twice
    int debounce = 0;

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
                limelight.getEntry("pipeline").setNumber(1);
            } catch (Exception e) {
            }
        }).start();

        AutoBuilder.configureHolonomic(
        SwerveS::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
            new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
            Constants.DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(true,true) // Default path replanning config. See the API for the options here
        ),
        SwerveS::getAlliance,
        this // Reference to this subsystem to set requirements
        );

        SmartDashboard.putData("Field", robotField);
    }
    
    public void zeroHeading() {
        debounce = 0;
        gyro.reset();
    }
    
    public static double getHeading() {
        return -1*Math.IEEEremainder(gyro.getAngle() + (getAlliance() ? 180 : 0),360); //modulus
    }
    
    public static Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public static boolean getAutoLock() {
        return autoLock;
    }
 
    @Override
    public void periodic() {
        periodicUpdateCycle +=1;

        if (limelight.getEntry("pipeline").getDouble(0) != 1) {
            limelight.getEntry("pipeline").setNumber(1);
        }

        if (getAlliance() && limelight.getEntry("priorityid").getDouble(0.0) != 4.0) {
            limelight.getEntry("priorityid").setNumber(4);
        } else if (!getAlliance() && limelight.getEntry("priorityid").getDouble(0.0) != 7.0) {
            limelight.getEntry("priorityid").setNumber(7);
        }
        

        /*if(periodicUpdateCycle%5 == 0){
            updatePoseEstimatorWithVisionBotPose();
        }*/
        redIsAlliance = getAlliance();
        //puts values to smartDashboard
        SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        SmartDashboard.putNumber("FrontLeft Abs Encoder", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FrontRight Abs Encoder", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BackLeft Abs Encoder", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BackRight Abs Encoder", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("xError", xError);
        SmartDashboard.putBoolean("Auto Lock", autoLock);
        SmartDashboard.putBoolean("Red is Alliance", getAlliance());
        zAccel = gyro.getRawAccelZ();
        SmartDashboard.putNumber("Position X (getPose)", getPose().getX());
        SmartDashboard.putNumber("Position Y (getPose)", getPose().getY());
        SmartDashboard.putNumber("Robot Heading (getPose)", getPose().getRotation().getDegrees());
        


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
        navXDisconnectProtocol();
        if (periodicUpdateCycle %10 == 0){
            DataHandler.logData(periodicUpdateCycle);
        }
    }


    public static double getZAccel(){
        return zAccel;
    }
    public static double getZDistance(){
        return limelight.getEntry("ty").getDouble(0.0);
    }
    public static double getXError() {
        // bounds xError between -5 and 5 (normal range of xError is -30 to 30)
        double bounded = xError/6 + Math.copySign(0.9999, xError); //adds 0.9999 to reduce dead area range once we square
        return bounded*Math.abs(bounded);


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
    /**
     *Returns whether the alliance is red.
     */
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
        
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.limelightName);
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
    /**
     * Function that sets the robot to robot relative (and then sets the leds to a pattern showing that the navx has disconnected) if the navx has disconnected
     */
    public void navXDisconnectProtocol(){
        if (gyro.isConnected() && debounce == 0){
            fieldOriented = true;
            return;
        }
        else{
            debounce = -1;
            fieldOriented = false;
        }
    }

    public static void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    public static void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }
}
