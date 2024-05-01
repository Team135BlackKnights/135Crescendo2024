package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.DriveSimConstants;

public class SwerveModuleSim {
    private static double 
    drivePos = 0,
    steerPos = 0,
    driveVelocity = 0;

    private static double dt = .02;
    
    private static Matrix<N2,N1> 
    steerMotorPosMatrix = VecBuilder.fill(0,0),
    driveMotorPosMatrix = VecBuilder.fill(0,0);
    
   
    private static Matrix<N1,N1> 
    driveMotorVoltageMatrix = VecBuilder.fill(0),
    steerMotorVoltageMatrix = VecBuilder.fill(0);
    
    

    private static LinearSystem<N2, N1, N1> 
    steerMotorPosLinearSystem,
    driveMotorPosLinearSystem;
    
    public SwerveModuleSim(double[] driveKsKvKa, double[] steerKsKvKa, double updateRate){
        
        //We should need one position linear system and one velocity linear system
        //TODO: Custom update rate
        steerMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(steerKsKvKa[1], steerKsKvKa[2]); 
        driveMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(driveKsKvKa[1], driveKsKvKa[2]);
        
        
    }
    /*Outputs as drivePos, steerPos, driveVelocity. Call in periodic */
    public static void updateModuleStates(double driveMotorPercent, double steerMotorPercent){
        
        //Create voltage matrices
        driveMotorVoltageMatrix = VecBuilder.fill(driveMotorPercent*DriveSimConstants.motorMaxVoltage);
        steerMotorVoltageMatrix = VecBuilder.fill(steerMotorPercent*DriveSimConstants.motorMaxVoltage);


        
        //Input [position, velocity] and voltage matrices
        driveMotorPosMatrix = driveMotorPosLinearSystem.calculateX(driveMotorPosMatrix, driveMotorVoltageMatrix, dt);
        steerMotorPosMatrix = steerMotorPosLinearSystem.calculateX(steerMotorPosMatrix, steerMotorVoltageMatrix, dt);
        
        //Converts those matrices to actual simulated inputs (since it moduluses at 2pi, check if this is right)
        drivePos = driveMotorPosMatrix.get(0,0)%(2*Math.PI);
        steerPos = steerMotorPosMatrix.get(0,0)%(2*Math.PI);
        driveVelocity = driveMotorPosMatrix.get(1,0);

        //Redoes the matrices to hold the modulused values back into the [position, velocity] matrices
        driveMotorPosMatrix = VecBuilder.fill(drivePos, driveMotorPosMatrix.get(1,0));
        steerMotorPosMatrix = VecBuilder.fill(steerPos, steerMotorPosMatrix.get(1,0));
        
    }

    public static SwerveModuleState getSimModuleState(){
        return new SwerveModuleState(driveVelocity, new Rotation2d(steerPos));
    }

    public static SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(drivePos, new Rotation2d(steerPos));
    }

    public static double getDrivePosRadiansSim(){
        return drivePos;
    }

    public static double getSteerPosRadiansSim(){
        return steerPos;
    }

    public static double getDriveVelocityRadiansSim(){
        return driveVelocity;
    }
}
