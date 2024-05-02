package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
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
    
    
   
    private static Matrix<N1,N1> 
    driveMotorVoltageMatrix = VecBuilder.fill(0),
    turningMotorVoltageMatrix = VecBuilder.fill(0),
    driveMotorVelocityMatrix = VecBuilder.fill(0),
    turningMotorVelocityMatrix = VecBuilder.fill(0);

    private static LinearSystem<N1,N1,N1>
    driveMotorVelocityLinearSystem,
    turningMotorVelocityLinearSystem;
    
    public SwerveModuleSim(double[] driveKsKvKa, double[] turningKsKvKa, double updateRate){
        
        //We use two flywheel linear sytems now
        //TODO: Custom update rate
        driveMotorVelocityLinearSystem = LinearSystemId.identifyVelocitySystem(driveKsKvKa[1], driveKsKvKa[2]);
        turningMotorVelocityLinearSystem = LinearSystemId.identifyVelocitySystem(turningKsKvKa[1], turningKsKvKa[2]);
    }
    /*Outputs as drivePos, turningPos, driveVelocity. Call in periodic */
    public void updateModuleState(){
        
        //Create voltage matrices
        driveMotorVoltageMatrix = VecBuilder.fill(driveMotorVoltage);
        turningMotorVoltageMatrix = VecBuilder.fill(turningMotorVoltage);


        
        //Input [position, velocity] and voltage matrices
        driveMotorVelocityMatrix = driveMotorVelocityLinearSystem.calculateX(driveMotorVelocityMatrix, driveMotorVoltageMatrix, dt);
        turningMotorVelocityMatrix = turningMotorVelocityLinearSystem.calculateX(turningMotorVelocityMatrix, turningMotorVoltageMatrix, dt);
        //Converts those matrices to actual simulated inputs (since it moduluses at 2pi, check if this is right)
        driveVelocity = driveMotorVelocityLinearSystem.calculateY(driveMotorVelocityMatrix, driveMotorVoltageMatrix).get(0,0);
        turningVelocity = turningMotorVelocityLinearSystem.calculateY(driveMotorVelocityMatrix, driveMotorVoltageMatrix).get(0,0);
        drivePos = ((driveVelocity*1.02)%(2*Math.PI)-Math.PI);
        turningPos = ((driveVelocity*1.02)%(2*Math.PI)-Math.PI);
        //Redoes the matrices to hold the modulused values back into the [position, velocity] matrices
        //driveMotorPosMatrix = VecBuilder.fill(drivePos, driveMotorPosMatrix.get(1,0));
        //turningMotorPosMatrix = VecBuilder.fill(turningPos, turningMotorPosMatrix.get(1,0));
        
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
