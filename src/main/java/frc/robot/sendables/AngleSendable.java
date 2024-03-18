package frc.robot.sendables;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
public class AngleSendable implements Sendable{
    @Override 
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Angle");

    }
}
