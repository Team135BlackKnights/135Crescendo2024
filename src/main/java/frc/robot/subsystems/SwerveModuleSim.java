package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.DriveSimConstants;
import edu.wpi.first.wpilibj.Notifier;

public class SwerveModuleSim {
    
    private static double dt = .02;
    
    private static Matrix<N1,N1> 
    driveMotorSpeedMatrix = VecBuilder.fill(0),
    steerMotorSpeedMatrix = VecBuilder.fill(0),
    steerMotorPosMatrix = VecBuilder.fill(0),
    driveMotorPosMatrix = VecBuilder.fill(0);
    
   
    private static Matrix<N1,N1> 
    driveMotorVoltageMatrix = VecBuilder.fill(0),
    steerMotorVoltageMatrix = VecBuilder.fill(0);
    
    
    private static LinearSystem<N1, N1, N1> 
    driveMotorSpeedLinearSystem,
    steerMotorSpeedLinearSystem;

    private static LinearSystem<N2, N1, N1> 
    steerMotorPosLinearSystem,
    driveMotorPosLinearSystem;
    
    public SwerveModuleSim(double driveKa, double steerKa, double driveKv, double steerKv, double updateRate){
        
        //We should need one position linear system and one velocity linear system
        //TODO: Custom update rate
        steerMotorSpeedLinearSystem = LinearSystemId.identifyVelocitySystem(steerKv, steerKa);
        driveMotorSpeedLinearSystem = LinearSystemId.identifyVelocitySystem(steerKv, steerKa);
        steerMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(driveKv, driveKa); 
        driveMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(steerKv, steerKa);
        
        
    }
    /*Outputs as drivePos, steerPos, driveSpeed */
    public static double[] updateModuleStates(double driveMotorPercent, double steerMotorPercent){
        
        //Update voltages
        driveMotorVoltageMatrix = VecBuilder.fill(driveMotorPercent*DriveSimConstants.motorMaxVoltage);
        steerMotorVoltageMatrix = VecBuilder.fill(steerMotorPercent*DriveSimConstants.motorMaxVoltage);
        
        //Update the velocity linear matrixes
        driveMotorSpeedMatrix = driveMotorSpeedLinearSystem.calculateX(driveMotorSpeedMatrix, driveMotorVoltageMatrix, dt);
        steerMotorSpeedMatrix = steerMotorSpeedLinearSystem.calculateX(steerMotorSpeedMatrix, steerMotorVoltageMatrix, dt);

        //Create new [position, velocity] matrices
        Matrix<N2,N1> driveMotorPosStates = VecBuilder.fill(steerMotorPosMatrix.get(0, 0), steerMotorSpeedMatrix.get(0,0));
        Matrix<N2,N1> steerMotorPosStates = VecBuilder.fill(driveMotorPosMatrix.get(0, 0), driveMotorSpeedMatrix.get(0,0));
        
        //Input updated vector into position matrices
        driveMotorPosMatrix = VecBuilder.fill(driveMotorPosLinearSystem.calculateX(driveMotorPosStates, driveMotorVoltageMatrix, dt).get(0,0));
        steerMotorPosMatrix = VecBuilder.fill(steerMotorPosLinearSystem.calculateX(steerMotorPosStates, driveMotorVoltageMatrix, dt).get(0,0));
        
        //Moduluses the positions, outputs all 3
        return new double[]{driveMotorPosMatrix.get(0,0)%(2*Math.PI), steerMotorPosMatrix.get(0,0)%(2*Math.PI), driveMotorSpeedMatrix.get(0,0)};
    }

}
