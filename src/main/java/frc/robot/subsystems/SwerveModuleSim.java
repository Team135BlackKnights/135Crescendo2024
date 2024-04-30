package frc.robot.subsystems;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.DriveSimConstants;
public class SwerveModuleSim {
    public SwerveModuleSim(double driveKa, double steerKa, double driveKv, double steerKv){
        //We should need one position linear system and one velocity linear system
        final LinearSystem<N1, N1, N1> steerMotorLinearSystem = LinearSystemId.identifyVelocitySystem(steerKv, steerKa);
        final LinearSystem<N2, N1, N1> driveMotorLinearSystem = LinearSystemId.identifyPositionSystem(driveKv, driveKa); 
        
    }
}
