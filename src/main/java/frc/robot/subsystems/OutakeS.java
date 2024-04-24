package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Volts;

public class OutakeS extends SubsystemBase {
    //motor declarations
   
    public PIDController shooterPID = new PIDController(Constants.OutakeConstants.kP, 0, 0);
    // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
    // those calculated by SysId
    private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          Constants.OutakeConstants.kSVolts,
          Constants.OutakeConstants.kVVoltSecondsPerRotation,
          Constants.OutakeConstants.kAVoltSecondsSquaredPerRotation);
  
    public CANSparkMax topFlywheel = new CANSparkMax(Constants.OutakeConstants.topFlywheel, MotorType.kBrushless);
    public CANSparkMax bottomFlywheel = new CANSparkMax(Constants.OutakeConstants.bottomFlywheel, MotorType.kBrushless);
    
    //encoder declarations
    public static RelativeEncoder 
    topFlywheelEncoder,
    bottomFlywheelEncoder;

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                topFlywheel.setVoltage(volts.in(Volts));
              },
          null // No log consumer, since data is recorded by URCL
    , this
        )
      );


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
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Speed", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Speed", bottomFlywheelEncoder.getVelocity());
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
   * 
   * @param shooterSpeed in RPM
   * @return Command
  */
  public Command runShooter(DoubleSupplier shooterSpeed) {
    // Run shooter wheel at the desired speed using a PID controller and feedforward.
    return run(() -> {
        topFlywheel.setVoltage(
            shooterPID.calculate(topFlywheelEncoder.getVelocity(), shooterSpeed.getAsDouble()) 
                + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));
        bottomFlywheel.setVoltage(
            shooterPID.calculate(bottomFlywheelEncoder.getVelocity(), shooterSpeed.getAsDouble())
                + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));
        })
        .finallyDo(
            () -> {
              topFlywheel.stopMotor();
              bottomFlywheel.stopMotor();
            })
        .withName("runShooter");
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
    /**
     * Runs motors at specific RPM.
     * 
     * @param speed The RPM of the flywheels.
     */
    public void setRPM(double rpm){
        topFlywheel.setVoltage(
            shooterPID.calculate(topFlywheelEncoder.getVelocity(), rpm));
               // + m_shooterFeedforward.calculate(rpm));
        bottomFlywheel.setVoltage(
            shooterPID.calculate(bottomFlywheelEncoder.getVelocity(), rpm));
                //+ m_shooterFeedforward.calculate(rpm));
    }
    /*
     **For shooting amp
     */
    public void setIndividualFlywheelSpeeds(double topWheelSpeed, double bottomWheelSpeed){
        topFlywheel.setVoltage(
            shooterPID.calculate(topFlywheelEncoder.getVelocity(), topWheelSpeed));
                //+ m_shooterFeedforward.calculate(topWheelSpeed));
        bottomFlywheel.setVoltage(
            shooterPID.calculate(bottomFlywheelEncoder.getVelocity(), bottomWheelSpeed));
                //+ m_shooterFeedforward.calculate(bottomWheelSpeed));
    }
}
