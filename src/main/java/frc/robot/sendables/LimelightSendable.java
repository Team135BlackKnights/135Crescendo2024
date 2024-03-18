package frc.robot.sendables;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.SwerveS;
public class LimelightSendable implements Sendable{
    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Limelight");
        builder.addDoubleProperty("Position X (getPose)", SwerveS.poseXSupplier, null);
        builder.addDoubleProperty("Position Y (getPose)", SwerveS.poseYSupplier,null);
        builder.addDoubleProperty("Robot Heading (getPose)", SwerveS.getPoseHeadingSupplier, null);
        builder.addDoubleProperty("April Tag Distance", SwerveS.aprilTagDistanceSupplier, null);
        builder.addDoubleProperty("Odometry Distance", SwerveS.odometryDistanceSupplier, null);
    }
}
