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

public class SwerveModuleSim {
    private double 
    drivePos = 0,
    turningPos = 0,
    driveVelocity = 0,
    turningVelocity = 0,
    driveMotorVoltage = 0,
    turningMotorVoltage = 0;
    private static double dt = .02;
    
    private static Matrix<N2,N1> 
    turningMotorPosMatrix = VecBuilder.fill(0,0),
    driveMotorPosMatrix = VecBuilder.fill(0,0);
    
   
    private static Matrix<N1,N1> 
    driveMotorVoltageMatrix = VecBuilder.fill(0),
    turningMotorVoltageMatrix = VecBuilder.fill(0);
    
    

    private static LinearSystem<N2, N1, N1> 
    turningMotorPosLinearSystem,
    driveMotorPosLinearSystem;
    
    public SwerveModuleSim(double[] driveKsKvKa, double[] turningKsKvKa, double updateRate){
        
        //We should need one position linear system and one velocity linear system
        //TODO: Custom update rate
        turningMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(turningKsKvKa[1], turningKsKvKa[2]); 
        driveMotorPosLinearSystem = LinearSystemId.identifyPositionSystem(driveKsKvKa[1], driveKsKvKa[2]);
        
        
    }
    /*Outputs as drivePos, turningPos, driveVelocity. Call in periodic */
    public void updateModuleState(){
        
        //Create voltage matrices
        driveMotorVoltageMatrix = VecBuilder.fill(driveMotorVoltage);
        turningMotorVoltageMatrix = VecBuilder.fill(turningMotorVoltage);


        
        //Input [position, velocity] and voltage matrices
        driveMotorPosMatrix = driveMotorPosLinearSystem.calculateX(driveMotorPosMatrix, driveMotorVoltageMatrix, dt);
        turningMotorPosMatrix = turningMotorPosLinearSystem.calculateX(turningMotorPosMatrix, turningMotorVoltageMatrix, dt);
        
        //Converts those matrices to actual simulated inputs (since it moduluses at 2pi, check if this is right)
        drivePos = driveMotorPosMatrix.get(0,0)%(2*Math.PI);
        turningPos = turningMotorPosMatrix.get(0,0)%(2*Math.PI);
        driveVelocity = driveMotorPosMatrix.get(1,0);
        turningVelocity = turningMotorPosMatrix.get(1,0);

        //Redoes the matrices to hold the modulused values back into the [position, velocity] matrices
        driveMotorPosMatrix = VecBuilder.fill(drivePos, driveMotorPosMatrix.get(1,0));
        turningMotorPosMatrix = VecBuilder.fill(turningPos, turningMotorPosMatrix.get(1,0));
        
    }

    public SwerveModuleState getSimModuleState(){
        return new SwerveModuleState(driveVelocity, new Rotation2d(turningPos));
    }

    public SwerveModulePosition getSwerveModuleSimPosition(){
        return new SwerveModulePosition(drivePos, new Rotation2d(turningPos));
    }

    public double getDrivePosRadiansSim(){
        return drivePos;
    }

    public double getTurningPosRadiansSim(){
        return turningPos;
    }

    public double getDriveVelocityMetersPerSecond(){
        return driveVelocity;
    }
    
    public double getTurningVelocityMetersPerSecond(){
        return turningVelocity;
    }
    

    public void resetEncoders(){
        drivePos = 0;
        turningPos = 0;
    }
    public void updateVoltage(double driveVolts, double steerVolts){
        driveMotorVoltage = driveVolts;
        turningMotorVoltage = steerVolts;
    }
}
