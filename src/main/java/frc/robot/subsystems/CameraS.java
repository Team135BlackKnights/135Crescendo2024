package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class CameraS extends SubsystemBase {
    public VisionSystemSim visionSim;
    public final PhotonPoseEstimator frontEstimator;
    public final PhotonPoseEstimator rightEstimator;
    public final PhotonPoseEstimator leftEstimator;
    public final PhotonPoseEstimator backEstimator;
    public final PhotonPoseEstimator[] camEstimates;
    public final PhotonCamera[] cams;
    public double 
        frontLastEstTimestamp = 0f,
        rightLastEstTimestamp = 0f,
        leftLastEstTimestamp = 0f,
        backLastEstTimestamp = 0f;
    private PhotonCamera frontCam;
    private PhotonCamera rightCam;
    private PhotonCamera leftCam;
    private PhotonCamera backCam;
    public CameraS() {
     //   frontCam = new PhotonCamera(Constants.VisionConstants.frontCamName);
        rightCam = new PhotonCamera(Constants.VisionConstants.rightCamName);
   //     leftCam = new PhotonCamera(Constants.VisionConstants.leftCamName);
    //    backCam = new PhotonCamera(Constants.VisionConstants.backCamName);
    //    frontCam.setPipelineIndex(0);
        rightCam.setPipelineIndex(0);
   //     leftCam.setPipelineIndex(0);
    //    backCam.setPipelineIndex(0);
        //POSITION CAMERAS (IMPORTANT)
        Translation3d frontPos = Constants.VisionConstants.frontCamTranslation3d;
        Translation3d rightPos = Constants.VisionConstants.rightCamTranslation3d;
        Translation3d leftPos = Constants.VisionConstants.leftCamTranslation3d;
        Translation3d backPos = Constants.VisionConstants.backCamTranslation3d;
        Rotation3d frontRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.frontCamPitch), 0); 
        Rotation3d rightRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.rightCamPitch), 0);
        Rotation3d leftRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.leftCamPitch), 0);
        Rotation3d backRot = new Rotation3d(0, Math.toRadians(Constants.VisionConstants.backCamPitch), 0);
        Transform3d robotToFront = new Transform3d(frontPos, frontRot); //transform
        Transform3d robotToRight = new Transform3d(rightPos, rightRot);
        Transform3d robotToLeft = new Transform3d(leftPos, leftRot);
        Transform3d robotToBack = new Transform3d(backPos, backRot);
    //sim
        frontEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,robotToFront);
        frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam,robotToRight);
        rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);    
        leftEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam,robotToLeft);
        leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);    
        backEstimator = new PhotonPoseEstimator(Constants.FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,robotToBack);
        backEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);   
      //  camEstimates = new PhotonPoseEstimator[]{frontEstimator,rightEstimator,leftEstimator,backEstimator};
        camEstimates = new PhotonPoseEstimator[]{rightEstimator};
        cams = new PhotonCamera[]{frontCam,rightCam,leftCam,backCam};
        
}
    public PhotonPipelineResult getLatestResult(PhotonCamera camera) {
        return camera.getLatestResult();
    }
 public Optional<EstimatedRobotPose> getEstimatedGlobalPose( PhotonPoseEstimator photonEstimator,PhotonCamera camera) {
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
        for (int i = 0; i <4; i++){
            PhotonPoseEstimator cEstimator = camEstimates[i];
            PhotonCamera cCam = cams[i];
            var visionEst = getEstimatedGlobalPose(cEstimator,cCam);
            visionEst.ifPresent(
                    est -> {
                        var estPose = est.estimatedPose.toPose2d();
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs(estPose,cEstimator,cCam);
    
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
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.FieldConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
