package frc.robot.sendables;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.IntakeS;
public class IntakeSendable implements Sendable{
    
    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Intake");
        builder.addBooleanProperty("Note in intake?", IntakeS.noteIsLoadedSupplier, null);
        builder.addBooleanProperty("Intake within bounds?", IntakeS.intakeWithinBoundsSupplier, null);
        builder.addDoubleProperty("Intake Angle", IntakeS.getIntakePositionSupplier, null);
        builder.addDoubleProperty("Deploy Intake", IntakeS.deployIntakeSupplier, null);
        builder.addDoubleProperty("Deploy Intake Abs", IntakeS.getIntakePositionSupplier, null);
    }
}
