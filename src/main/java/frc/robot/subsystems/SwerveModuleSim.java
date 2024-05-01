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
    public static double 
    drivePos = 0,
    steerPos = 0,
    driveSpeed = 0;

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
    
    public SwerveModuleSim(double[] driveKsKvKa, double[] steerKsKvKa, double updateRate){
        
        //We should need one position linear system and one velocity linear system
        //TODO: Custom update rate
        steerMotorSpeedLinearSystem = LinearSystemId.identifyVelocitySystem(steerKsKvKa[1], steerKsKvKa[2]);
        driveMotorSpeedLinearSystem = LinearSystemId.identifyVelocitySystem(driveKsKvKa[1], driveKsKvKa[2]);
        steerMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(steerKsKvKa[1], steerKsKvKa[2]); 
        driveMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(driveKsKvKa[1], driveKsKvKa[2]);
        
        
    }
    /*Outputs as drivePos, steerPos, driveSpeed */
    public static void updateModuleStates(double driveMotorPercent, double steerMotorPercent){
        
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
        drivePos = driveMotorPosMatrix.get(0,0)%(2*Math.PI);
        steerPos = steerMotorPosMatrix.get(0,0)%(2*Math.PI);
        driveSpeed = driveMotorSpeedMatrix.get(0,0);
    }

}
