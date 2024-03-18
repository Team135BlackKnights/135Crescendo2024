package frc.robot.sendables;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.HangS;
public class HangSendable implements Sendable{
    
    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Hang");
        builder.addDoubleProperty("Left Hang Encoder", HangS.leftHangSupplier , null);
        builder.addDoubleProperty("Right Hang Encoder", HangS.rightHangSupplier, null);

    }
}
