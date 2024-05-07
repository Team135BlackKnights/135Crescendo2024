package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.SwerveConstants;
public class SwerveModuleSim {
    private double 
    drivePos = 0,
    turningPos = 0,
    driveVelocity = 0,
    turningVelocity = 0,
    driveMotorRPM = 0,
    turningMotorRPM = 0,
    driveMotorVolts = 0,
    turningMotorVolts = 0;
    private static double dt = .02;
    double driveKs,
    turningKs;
    
   

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
    
    public SwerveModuleSim(double[] driveKpKsKvKa, double[] turningKpKsKvKa, double updateRate){
        
        driveKs = driveKpKsKvKa[2];
        turningKs = driveKpKsKvKa[2];
        //TODO: Custom update rate
        //Velocity Systems
        driveMotorVelocityLinearSystem = LinearSystemId.identifyVelocitySystem(driveKpKsKvKa[1], driveKpKsKvKa[2]);
        turningMotorVelocityLinearSystem = LinearSystemId.identifyVelocitySystem(turningKpKsKvKa[1], turningKpKsKvKa[2]);

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
        driveMotorSim = new FlywheelSim(driveMotorVelocityLinearSystem, DCMotor.getNEO(1), SwerveConstants.kDriveMotorGearRatio);
        turningMotorSim = new FlywheelSim(turningMotorVelocityLinearSystem, DCMotor.getNEO(1), SwerveConstants.kTurningMotorGearRatio);
        driveMotorSystemLoop.setNextR(0);
        turningMotorSystemLoop.setNextR(0);
        
        //resets the initial states 
        driveMotorSystemLoop.reset(VecBuilder.fill(0));
        turningMotorSystemLoop.reset(VecBuilder.fill(0));

    }
    /*Outputs as drivePos, turningPos, driveVelocity. Call in periodic */
    public void updateModuleState(){
        //Predict next states
        driveMotorSystemLoop.predict(dt);
        turningMotorSystemLoop.predict(dt);
        //pulls current voltages
        driveMotorVolts = driveMotorSystemLoop.getU(0);
       // var error = turningMotorSystemLoop.getNextR().minus(x);
       // error.set(0, 0, MathUtil.angleModulus(error.get(0, 0)));
      //  turningMotorVolts = turningMotorSystemLoop.getK().times(error);
        turningMotorVolts = turningMotorSystemLoop.getU(0);

        //sets current states as input
        driveMotorSim.setInput(driveMotorVolts);
        turningMotorSim.setInput(turningMotorVolts);

        //updates them with the dt
        driveMotorSim.update(dt);
        turningMotorSim.update(dt);

        

        
        //Outputs velocity, converts to rad per sec 
        driveVelocity = driveMotorSim.getAngularVelocityRPM();
        turningVelocity = turningMotorSim.getAngularVelocityRPM();
        
        //Kinematics wants wheel speed in meters per second. V = angular speed * radius
        driveVelocity = driveVelocity*SwerveConstants.kWheelDiameter/2;

        //Physics Formula: p = v0t+p0
        drivePos += driveVelocity*dt;
        turningPos += turningPos*dt;

        //so it doesn't freak out when it equals 0
        if (drivePos == 0){
            drivePos = .001;
        }
        if (turningPos == 0){
            turningPos = .001;
        }
        //Modulus the positions by 2pi, subtract pi to make them accurate for the range of the actual kinematics (-pi to pi)
        drivePos = (drivePos%(2*Math.PI))-Math.PI;
        turningPos = (drivePos%(2*Math.PI))-Math.PI;

        
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
        //Ks is in volts/(m/s), so 1/ks give us (m/s)/v
        driveMotorRPM = driveVolts/driveKs;
        turningMotorRPM = steerVolts/turningKs;
        driveMotorSystemLoop.setNextR(driveMotorRPM);
        turningMotorSystemLoop.setNextR(turningMotorRPM);
    }
}
