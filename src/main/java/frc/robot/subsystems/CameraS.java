package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OutakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DriveConstants;

public class CameraS extends SubsystemBase {
    public VisionSystemSim visionSim;
   // public final PhotonPoseEstimator frontEstimator;
    public final PhotonPoseEstimator rightEstimator;
   // public final PhotonPoseEstimator leftEstimator;
    public final PhotonPoseEstimator backEstimator;
    public final PhotonPoseEstimator[] camEstimates;
    public final PhotonCamera[] cams;
    public double 
        frontLastEstTimestamp = 0f,
        rightLastEstTimestamp = 0f,
        leftLastEstTimestamp = 0f,
        backLastEstTimestamp = 0f;
    public static double backCamXError = 0f;
    static double distance = 0;
    static private PhotonCamera
        frontCam,
        rightCam,
        leftCam;
    private static PhotonCamera backCam;
    public CameraS() {
        //frontCam = new PhotonCamera(Constants.VisionConstants.frontCamName);
        rightCam = new PhotonCamera(Constants.VisionConstants.rightCamName);
        //leftCam = new PhotonCamera(Constants.VisionConstants.leftCamName);
        backCam = new PhotonCamera(Constants.VisionConstants.backCamName);
        //frontCam.setPipelineIndex(0);
        rightCam.setPipelineIndex(0);
        //leftCam.setPipelineIndex(0);
        backCam.setPipelineIndex(0);
        //POSITION CAMERAS (IMPORTANT)
        Translation3d frontPos = Constants.VisionConstants.frontCamTranslation3d;
        Translation3d rightPos = Constants.VisionConstants.rightCamTranslation3d;
        Translation3d leftPos = Constants.VisionConstants.leftCamTranslation3d;
        Translation3d backPos = Constants.VisionConstants.backCamTranslation3d;
        Rotation3d frontRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.frontCamPitch), 0); 
        Rotation3d rightRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.rightCamPitch), -90);
        Rotation3d leftRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.leftCamPitch), 90);
        Rotation3d backRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.backCamPitch), 180);
        Transform3d robotToFront = new Transform3d(frontPos, frontRot); //transform
        Transform3d robotToRight = new Transform3d(rightPos, rightRot);
        Transform3d robotToLeft = new Transform3d(leftPos, leftRot);
        Transform3d robotToBack = new Transform3d(backPos, backRot);
    //sim
        //frontEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,robotToFront);
        //frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        rightEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam,robotToRight);
        rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);    
        //leftEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam,robotToLeft);
        //leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);    
        backEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,robotToBack);
        backEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE); 
        //camEstimates = new PhotonPoseEstimator[]{frontEstimator,rightEstimator,leftEstimator,backEstimator};
        camEstimates = new PhotonPoseEstimator[]{rightEstimator,backEstimator};
        //cams = new PhotonCamera[]{frontCam,rightCam,leftCam,backCam};
        cams = new PhotonCamera[]{rightCam,backCam};
        
}
public static boolean aprilTagVisible() {
    return backCam.getLatestResult().hasTargets();
}
    public PhotonPipelineResult getLatestResult(PhotonCamera camera) {
        return camera.getLatestResult();
    }
 public Optional<EstimatedRobotPose> getEstimatedGlobalPose( PhotonPoseEstimator photonEstimator,PhotonCamera camera,Pose2d prevEstPose2d) {
        photonEstimator.setReferencePose(prevEstPose2d);
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = false;
        if (camera.getName() == Constants.VisionConstants.frontCamName){
            newResult = Math.abs(latestTimestamp - frontLastEstTimestamp) > 1e-5;
            if (newResult) frontLastEstTimestamp = latestTimestamp;
        } else if (camera.getName() == Constants.VisionConstants.rightCamName){
            newResult = Math.abs(latestTimestamp - rightLastEstTimestamp) > 1e-5;
            if (newResult) rightLastEstTimestamp = latestTimestamp;
        } else if (camera.getName() == Constants.VisionConstants.leftCamName){
            newResult = Math.abs(latestTimestamp - leftLastEstTimestamp) > 1e-5;
            if (newResult) leftLastEstTimestamp = latestTimestamp;
        } else {
            newResult = Math.abs(latestTimestamp - backLastEstTimestamp) > 1e-5;
            if (newResult) backLastEstTimestamp = latestTimestamp;
        }
        
        return visionEst;
    }
    @Override
    public void periodic(){
        for (int i = 0; i <2; i++){
            PhotonPoseEstimator cEstimator = camEstimates[i];
            PhotonCamera cCam = cams[i];
            var visionEst = getEstimatedGlobalPose(cEstimator,cCam,SwerveS.getPose());
            visionEst.ifPresent(
                    est -> {
                        var estPose = est.estimatedPose.toPose2d();
                        Field2d cameraField = new Field2d();
                        cameraField.setRobotPose(estPose);
                        //SmartDashboard.putData(cameraField);
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs(estPose,cEstimator,cCam);
                        SmartDashboard.putString("CAMERAUPDATE", cCam.getName());
                        if (cCam.getName() == Constants.VisionConstants.backCamName){
                            var targets = backCam.getLatestResult().getTargets();
                            for (var target : targets){
                                if (SwerveS.getAlliance()){
                                    if (target.getFiducialId() == 4){
                                        backCamXError = target.getBestCameraToTarget().getX();
                                    }
                                }else{
                                    if (target.getFiducialId() == 7){
                                        backCamXError = target.getBestCameraToTarget().getX();
                                    }
                                }
                                
                            }
                        }
                        SwerveS.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        
                    });
        }

        
    }
    //FOR SWERVE ESTIMATION
     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPoseEstimator photonEstimator, PhotonCamera cam) {
        var estStdDevs = Constants.FieldConstants.kSingleTagStdDevs;
        var targets = getLatestResult(cam).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            if (tgt.getPoseAmbiguity() >.25) continue; //give zero F's about bad tags
            numTags++;

            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); //BIG.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.FieldConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
    public static double getDistanceFromSpeakerUsingRobotPose() {
        if (SwerveS.getAlliance()) {
            return SwerveS.robotPosition.getTranslation().getDistance(Constants.FieldConstants.redSpeaker);
        } else {
            return SwerveS.robotPosition.getTranslation().getDistance(Constants.FieldConstants.blueSpeaker);
        }
    }
    /**
     *Returns tX BETWEEN -5 and 5 of the backCam.
     */
public static double getXError() {
        // bounds xError between -5 and 5 (normal range of xError is -30 to 30)
        double bounded = CameraS.backCamXError/6 + Math.copySign(0.9999, CameraS.backCamXError); //adds 0.9999 to reduce dead area range once we square
        return bounded*Math.abs(bounded);
    }
public static boolean robotInRange() {
    return getDistanceFromSpeakerUsingRobotPose() > 1.9 && getDistanceFromSpeakerUsingRobotPose() < 2.2;
}
    public static double getDistanceFromSpeakerInMeters(){
        //resets value so it doesn't output last value

       /* if apriltag is detected, uses formula given here https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
       formula is d =(h2-h1)/tan(h2+h1)*/

       if (aprilTagVisible()){
        PhotonTrackedTarget target = backCam.getLatestResult().getBestTarget();

           // computing the angle
           double theta = Units.degreesToRadians(VisionConstants.backCamPitch + target.getBestCameraToTarget().getY()); // plus ty
           
       
           //computes distance
           distance = Units.inchesToMeters(FieldConstants.targetHeightoffFloorInches-VisionConstants.backCamTranslation3d.getZ())/Math.tan(theta);
       }
       else{return 0.0;}
       return distance;
   }

   public static double getDesiredShooterUpperBound() {
       double upperBoundHeight = 0;
       double upperBoundDistance = 0;
       if (getDistanceFromSpeakerUsingRobotPose() > 5) {
           upperBoundHeight = 1.09*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           upperBoundDistance = getDistanceFromSpeakerUsingRobotPose() - FieldConstants.speakerOpeningDepth - DriveConstants.kChassisLength;
       } else if (getDistanceFromSpeakerUsingRobotPose() > 4) {
           upperBoundHeight = 1.02*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           upperBoundDistance = getDistanceFromSpeakerUsingRobotPose() - FieldConstants.speakerOpeningDepth - DriveConstants.kChassisLength;
       } else if (getDistanceFromSpeakerUsingRobotPose() > 3) {
           upperBoundHeight = 0.916*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           upperBoundDistance = getDistanceFromSpeakerUsingRobotPose() - FieldConstants.speakerOpeningDepth - DriveConstants.kChassisLength;
       } else if (getDistanceFromSpeakerUsingRobotPose() > 2.4) {
           upperBoundHeight = 0.82*FieldConstants.speakerUpperLipHeight-FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           upperBoundDistance = getDistanceFromSpeakerUsingRobotPose() - FieldConstants.speakerOpeningDepth - DriveConstants.kChassisLength;
       } else {
           return 44;
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
           lowerBoundHeight = 1.1*FieldConstants.speakerLowerLipHeight + FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           lowerBoundDistance = getDistanceFromSpeakerUsingRobotPose() - DriveConstants.kChassisLength;
       } else if (getDistanceFromSpeakerUsingRobotPose() > 4) {
           lowerBoundHeight = 1.036*FieldConstants.speakerLowerLipHeight + FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           lowerBoundDistance = getDistanceFromSpeakerUsingRobotPose() - DriveConstants.kChassisLength;
       } else if (getDistanceFromSpeakerUsingRobotPose() > 3) {
           lowerBoundHeight = 0.96*FieldConstants.speakerLowerLipHeight + FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           lowerBoundDistance = getDistanceFromSpeakerUsingRobotPose() - DriveConstants.kChassisLength;
       } else if (getDistanceFromSpeakerUsingRobotPose() > 2.4) {
           lowerBoundHeight = 0.95*FieldConstants.speakerLowerLipHeight + FieldConstants.noteHeight-OutakeConstants.shooterHeight;
           lowerBoundDistance = getDistanceFromSpeakerUsingRobotPose() - DriveConstants.kChassisLength;
       } else {
           return 42;
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
       double angle = (getDesiredShooterUpperBound() + getDesiredShooterLowerBound())/2;
       if (angle > 42) angle = 43;
       return angle;
   }
}
