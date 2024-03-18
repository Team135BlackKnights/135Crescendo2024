package frc.robot.sendables;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.SwerveS;

public class SwerveSendable implements Sendable{
    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drivetrain");
        builder.addBooleanProperty("Alliance", SwerveS.redisAllianceSupplier, null);
        builder.addBooleanProperty("Auto Lock", SwerveS.autoLockSupplier, null);
        builder.addDoubleProperty("Front Left Position", SwerveS.frontLeftAbsEncoderSupplier, null);
        builder.addDoubleProperty("Front Right Position", SwerveS.frontRightAbsEncoderSupplier, null);
        builder.addDoubleProperty("Back Left Position", SwerveS.backLeftAbsEncoderSupplier, null);
        builder.addDoubleProperty("Back Right Position", SwerveS.backRightAbsEncoderSupplier, null);
        builder.addDoubleProperty("Robot Heading", SwerveS.robotHeadingSupplier, null);
        builder.addDoubleProperty("X Error", SwerveS.xErrorSupplier, null);
    }
}
