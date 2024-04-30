package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.DriveSimConstants;
public class SwerveModuleSim {
    private double driveMaxVoltage = 12;
    private double steerMaxVoltage = 12;
    LinearSystem<N1, N1, N1> steerMotorLinearSystem;
    LinearSystem<N2, N1, N1> driveMotorLinearSystem;
    public SwerveModuleSim(CANSparkMax drive, CANSparkMax steer, double[] steerKvKaKs, double[] driveKvKaKs){
        //We should need one position linear system and one velocity linear system
        steerMotorLinearSystem = LinearSystemId.identifyVelocitySystem(steerKvKaKs[0], steerKvKaKs[1]);
        driveMotorLinearSystem = LinearSystemId.identifyPositionSystem(driveKvKaKs[0], driveKvKaKs[1]); 
        
    }
    public void setDrive(double percentVoltage){
        double voltage = driveMaxVoltage*percentVoltage;
        double sign = Math.signum(voltage);
        
    }
}
