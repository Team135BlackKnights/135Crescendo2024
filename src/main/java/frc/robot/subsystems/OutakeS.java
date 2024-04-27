package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class OutakeS extends SubsystemBase {
    //motor declarations
    public static boolean SysIDTestRunning = false;

    public CANSparkMax topFlywheel = new CANSparkMax(Constants.OutakeConstants.topFlywheel, MotorType.kBrushless);
    public CANSparkMax bottomFlywheel = new CANSparkMax(Constants.OutakeConstants.bottomFlywheel, MotorType.kBrushless);
    //encoder declarations
    public static RelativeEncoder 
    topFlywheelEncoder,
    bottomFlywheelEncoder;
    Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
    Measure<Voltage> holdVoltage = Volts.of(7);
    Measure<Time> timeout = Seconds.of(10);
    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(rampRate,holdVoltage,timeout,(state) -> Logger.recordOutput("SysIdTestState", state.toString())        ),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                topFlywheel.setVoltage(volts.in(Volts));
                bottomFlywheel.setVoltage(volts.in(Volts));
              },
          null // No log consumer, since data is recorded by URCL
    , this
        )
      );

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite
    private final static LinearSystem<N1, N1, N1> m_topFlywheelPlant =
        LinearSystemId.identifyVelocitySystem(Constants.OutakeConstants.kVVoltSecondsPerRotation, Constants.OutakeConstants.kAVoltSecondsSquaredPerRotation);
    //to reject noise, we use a kalman filter.
    private final static KalmanFilter<N1,N1,N1> m_topObserver = 
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_topFlywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is in St.Devs, HIGHER = trust more.
            VecBuilder.fill(0.01), // How accurate we think each encoder value matters in St.Devs. 
            .02 //never touch, rio runs at 20 ms.
            );
     // A LQR is basically our PID controller, for ✨State Space✨
    private final static LinearQuadraticRegulator<N1, N1, N1> m_topController =
    new LinearQuadraticRegulator<>(
        m_topFlywheelPlant,
        VecBuilder.fill(8.0), /* qelms. velocity error tolerances, in meters per second. Decrease this to more
        heavily penalize state excursion, or make the controller behave more aggressively. In
        this example we weight position much more highly than velocity, but this can be
        tuned to balance the two.*/
        VecBuilder.fill(12.0), // voltage tolerance
        0.020);
    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final static LinearSystemLoop<N1, N1, N1> m_topLoop =
    new LinearSystemLoop<>(m_topFlywheelPlant, m_topController, m_topObserver, 12.0, 0.020); //max physical voltage, not applied.
    
    
    private final static LinearSystem<N1, N1, N1> m_bottomFlywheelPlant =LinearSystemId.identifyVelocitySystem(Constants.OutakeConstants.kVVoltSecondsPerRotation, Constants.OutakeConstants.kAVoltSecondsSquaredPerRotation);
    private final static KalmanFilter<N1,N1,N1> m_bottomObserver = new KalmanFilter<>(Nat.N1(),Nat.N1(),m_bottomFlywheelPlant,VecBuilder.fill(3.0),VecBuilder.fill(0.01), .02 );
    private final static LinearQuadraticRegulator<N1, N1, N1> m_bottomController =
    new LinearQuadraticRegulator<>(m_bottomFlywheelPlant,VecBuilder.fill(8.0),VecBuilder.fill(12.0),0.020);
    private final static LinearSystemLoop<N1, N1, N1> m_bottomLoop = new LinearSystemLoop<>(m_bottomFlywheelPlant, m_bottomController, m_bottomObserver, 12.0, 0.020);
    public OutakeS() {
        //checks to see if motors are inverted
        topFlywheel.setInverted(Constants.OutakeConstants.topFlywheelReversed);
        bottomFlywheel.setInverted(Constants.OutakeConstants.bottomFlywheelReversed);
        //sets idle mode on motors
        topFlywheel.setIdleMode(IdleMode.kBrake);
        bottomFlywheel.setIdleMode(IdleMode.kBrake);
        //assigns values to encoders
        topFlywheelEncoder = topFlywheel.getEncoder();
        bottomFlywheelEncoder = bottomFlywheel.getEncoder();
        //makes encoders work with the gear ratio (basically means that one turn of the wheel will be one turn of the encoder)
        topFlywheelEncoder.setVelocityConversionFactor(Constants.OutakeConstants.flywheelGearRatio);
        bottomFlywheelEncoder.setVelocityConversionFactor(Constants.OutakeConstants.flywheelGearRatio);
        //sets changes to the motors' controllers
        topFlywheel.burnFlash();
        bottomFlywheel.burnFlash();

        // Reset our loop to make sure it's in a known state.
        //  (sparks have exact 20ms delay so not needed) m_controller.latencyCompensate(m_flywheelPlant, .02, .025); //sensor delay
        m_topLoop.reset(VecBuilder.fill(topFlywheelEncoder.getVelocity()));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Outake/Top Flywheel Speed",topFlywheelEncoder.getVelocity());
        Logger.recordOutput("Outake/Bottom Flywheel Speed",bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Top Flywheel Speed", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Speed", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Average Flywheel Speed", getAverageFlywheelSpeed());
    }
   /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
    public static double getAverageFlywheelSpeed() {
        //pulls the speed of the flywheels, used for pid loop
        double speed = topFlywheelEncoder.getVelocity() + bottomFlywheelEncoder.getVelocity();
        speed = speed/2;
        return speed;
    }

    public static double getFlywheelSpeedDifference() {
        return Math.abs(Math.abs(topFlywheelEncoder.getVelocity()) - Math.abs(bottomFlywheelEncoder.getVelocity()));
    }
    public static double getBottomSpeedError(){
        return m_bottomLoop.getError(0); //very low chance this is the wrong call
    }
    public static double getTopSpeedError(){
        return m_topLoop.getError(0);
    }
    /**
     * Runs motors at specific RPM.
     * 
     * @param speed The RPM of the flywheels.
     */
    public void setRPM(double rpm){
        //set setpoint
        m_topLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(rpm)));
        //correct for error
        m_topLoop.correct(VecBuilder.fill(getAverageFlywheelSpeed())); //maybe make two m_loops for top and bottom?
        m_topLoop.predict(0.02); //basically the same as .calculate
        double nextVoltage = m_topLoop.getU(0);
        topFlywheel.setVoltage(nextVoltage);
        bottomFlywheel.setVoltage(nextVoltage);
        /*topFlywheel.setVoltage(
            shooterPID.calculate(topFlywheelEncoder.getVelocity(), rpm)
                + m_shooterFeedforward.calculate(rpm));
        bottomFlywheel.setVoltage(
            shooterPID.calculate(bottomFlywheelEncoder.getVelocity(), rpm)
                + m_shooterFeedforward.calculate(rpm));*/
    }
    /*
     **For shooting amp
     */
    public void setIndividualFlywheelSpeeds(double topWheelSpeed, double bottomWheelSpeed){
        //set setpoint
        m_topLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(topWheelSpeed)));
        //correct for error
        m_topLoop.correct(VecBuilder.fill(topFlywheelEncoder.getVelocity()));
        m_topLoop.predict(0.02); //basically the same as .calculate
        double topNextVoltage = m_topLoop.getU(0);
        topFlywheel.setVoltage(topNextVoltage);

        m_bottomLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(bottomWheelSpeed)));
        m_bottomLoop.correct(VecBuilder.fill(bottomFlywheelEncoder.getVelocity()));
        m_bottomLoop.predict(0.02); 
        double bottomNextVoltage = m_bottomLoop.getU(0);
        bottomFlywheel.setVoltage(bottomNextVoltage);
    }
}
