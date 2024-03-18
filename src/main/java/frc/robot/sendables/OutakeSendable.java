package frc.robot.sendables;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.OutakeS;

public class OutakeSendable implements Sendable {
    @Override 
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Outake");
        builder.addDoubleArrayProperty(null, OutakeS.flyWheelSpeedsSupplier, null);
        builder.addDoubleProperty("Avg Flywheel Speed", OutakeS.averageFlyWheelSpeedSupplier, null);
        
    }
}
