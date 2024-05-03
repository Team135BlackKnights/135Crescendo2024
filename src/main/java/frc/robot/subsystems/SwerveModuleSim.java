package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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
    turningMotorVoltageMatrix = VecBuilder.fill(0);

    private static LinearSystem<N1,N1,N1>
    driveMotorVelocityLinearSystem,
    turningMotorVelocityLinearSystem;
    
    private static KalmanFilter<N1,N1,N1>
    driveMotorFilter,
    turningMotorFilter;

    private static LinearQuadraticRegulator<N1,N1,N1>
    driveMotorRegulator,
    turningMotorRegulator;

    private static LinearSystemLoop<N1,N1,N1>
    driveMotorSystemLoop,
    turningMotorSystemLoop;

    private static FlywheelSim
    driveMotorSim,
    turningMotorSim;
    
    public SwerveModuleSim(double[] driveKsKvKa, double[] turningKsKvKa, double updateRate){
        

        //TODO: Custom update rate
        //Velocity Systems
        driveMotorVelocityLinearSystem = LinearSystemId.identifyVelocitySystem(driveKsKvKa[1], driveKsKvKa[2]);
        turningMotorVelocityLinearSystem = LinearSystemId.identifyVelocitySystem(turningKsKvKa[1], turningKsKvKa[2]);

        //KalmanFilters
        driveMotorFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), driveMotorVelocityLinearSystem, VecBuilder.fill(3.0), VecBuilder.fill(0.01), dt);
        turningMotorFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), turningMotorVelocityLinearSystem, VecBuilder.fill(3.0), VecBuilder.fill(.001), dt);

        //LQRs
        driveMotorRegulator = new LinearQuadraticRegulator<>(driveMotorVelocityLinearSystem, VecBuilder.fill(8), VecBuilder.fill(12), dt);
        turningMotorRegulator = new LinearQuadraticRegulator<>(turningMotorVelocityLinearSystem, VecBuilder.fill(8), VecBuilder.fill(12), dt);

        //Linear System Loops
        driveMotorSystemLoop = new LinearSystemLoop<>(driveMotorVelocityLinearSystem, driveMotorRegulator, driveMotorFilter, 12.0, dt);
        turningMotorSystemLoop = new LinearSystemLoop<>(turningMotorVelocityLinearSystem, turningMotorRegulator, turningMotorFilter, 12.0, dt);

        //Flywheel Simulations
        driveMotorSim = new FlywheelSim(driveMotorVelocityLinearSystem, DCMotor.getNEO(1), dt);
        turningMotorSim = new FlywheelSim(turningMotorVelocityLinearSystem, DCMotor.getNEO(1), dt);
    }
    /*Outputs as drivePos, turningPos, driveVelocity. Call in periodic */
    public void updateModuleState(){
        
        //Create voltage matrices
        driveMotorVoltageMatrix = VecBuilder.fill(driveMotorVoltage);
        turningMotorVoltageMatrix = VecBuilder.fill(turningMotorVoltage);


        
       
        
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
