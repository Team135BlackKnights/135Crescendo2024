package frc.robot.subsystems;



//import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;


public class SwerveModule {

    private final SwerveModuleSim swerveModuleSim;



    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final ProfiledPIDController turningPIDController;
    private final PIDController drivePIDController; //not using profiled cuz no angles
    private SimpleMotorFeedforward turningFeedForward = null;
    
    private SimpleMotorFeedforward driveFeedForward = null;
    
    private final SparkAnalogSensor absoluteEncoder;
    //private final AnalogInput absoluteEncoder; // Use either AnalogInput or CANCoder depending on the absolute encoder
    //private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /**
     * 
     * @param driveMotorId Drive CANSparkMax Motor ID
     * @param turningMotorId Turning CANSparkMax Motor ID
     * @param driveMotorReversed True if Motor is Reversed
     * @param turningMotorReversed True if Motor is Reversed
     * @param absoluteEncoderId Turning Absolute Encoder ID
     * @param absoluteEncoderOffset Offset of Absolute Encoder in Radians
     * @param absoluteEncoderReversed True if Encoder is Reversed
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, double[] driveKpKsKvKa, double[] turningKpKsKvKa) {
       turningFeedForward = new SimpleMotorFeedforward(
        turningKpKsKvKa[1], turningKpKsKvKa[2], turningKpKsKvKa[3]);
        driveFeedForward = new SimpleMotorFeedforward(
        driveKpKsKvKa[1], driveKpKsKvKa[2], driveKpKsKvKa[3]);
       
       
       
        //sets values of the encoder offset and whether its reversed
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);
        //absoluteEncoder = new CANCoder(absoluteEncoderId);

        //declares motors
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        //checks to see if they're inverted
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        //sets the absolute encoder value (called this way because we have breakout boards in the motors)
        absoluteEncoder = turningMotor.getAnalog(Mode.kAbsolute);
        //relative encoder declarations
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        //sets motor idle modes to break
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        //accounts for gear ratios
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(Constants.SwerveConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveConstants.kTurningEncoderRPM2RadPerSec);
        //creates pidController, used exclusively for turning because that has to be precise
        //must test updated 
        turningPIDController = new ProfiledPIDController(turningKpKsKvKa[0], 0, 0,new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxTurningSpeedRadPerSec,Constants.DriveConstants.kTeleTurningMaxAcceleration));
        //makes the value loop around
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        drivePIDController = new PIDController(driveKpKsKvKa[0], 0, 0);
        if (Robot.isSimulation()){
            swerveModuleSim = new SwerveModuleSim(driveKpKsKvKa, turningKpKsKvKa, 3); 
        }
        else{
            swerveModuleSim = null;
        }
    }

    public double getDrivePosition() {
        //returns the position of the drive wheel
        if (Robot.isReal()){
            return driveEncoder.getPosition();
        }
        else{
            return swerveModuleSim.getDrivePosRadiansSim();
        }
        
    }

    public double getTurningPosition() {
        //returns the heading of the swerve module (turning motor position)
        if (Robot.isReal()){
            return getAbsoluteEncoderRad();
        }
        else{
            return swerveModuleSim.getTurningPosRadiansSim();
        }
        
    }

    public double getDriveVelocity() {
        //returns velocity of drive wheel
        if (Robot.isReal()){
            return driveEncoder.getVelocity();
        }
        else{
            return swerveModuleSim.getDriveVelocityMetersPerSecond();
        }
        
    }

    public double getTurningVelocity() {
        //returns velocity of turning motor
        if (Robot.isReal()){
            return turningEncoder.getVelocity();
        }
        else{
            return swerveModuleSim.getTurningVelocityMetersPerSecond();
        }
        
        
    }

    public void setTurningTest(double volts){
        turningMotor.setVoltage(volts);
    }
    public void setDriveTest(double volts){
        double turnOutput = turningPIDController.calculate(getAbsoluteEncoderRad(),0);
        turningMotor.setVoltage(turnOutput);
        driveMotor.setVoltage(volts);
    }
    public double getAbsoluteEncoderRad() {
        //gets the voltage and divides by the maximum voltage to get a percent, then multiplies that percent by 2pi to get a degree heading.
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage3V3(); // use 5V when plugged into RIO 3.3V when using breakout board
        angle *= 2 * Math.PI;
        //adds the offset in
        angle -= absoluteEncoderOffsetRad;
        /*this line of code is here because our pid loop is set to negative pi to pi,
        but our absolute encoders read from 0 to 2pi. 
        The line of code below basically says to add 2pi if an input is below 0 to get it in the range of 0 to 2pi*/
        angle += angle <= 0 ? 2*Math.PI : 0; 
        //subtracts pi to make the 0 to 2pi range back into pi to -pi
        angle -= Math.PI; //angle > Math.PI ? 2*Math.PI : 0;
        //if encoder is reversed multiply the input by negative one
        angle *= (absoluteEncoderReversed ? -1 : 1);
        return angle;
    } 

    /* public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle /= 360;
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        angle *= (absoluteEncoderReversed ? -1 : 1);
        return angle;
    } */

    public void resetEncoders() {
        //resets the encoders, (drive motor becomes zero, turning encoder becomes the module heading from the absolute encoder)
        if (Robot.isReal()){
            driveEncoder.setPosition(0);
            turningEncoder.setPosition(getAbsoluteEncoderRad()); 
        }
        else{
            swerveModuleSim.resetEncoders();
        }
        
    }

    public SwerveModulePosition getPosition() {
         //basically creates a new swervemoduleposition based on the current positions of the drive and turning encoders

        if (Robot.isReal()){
            return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(getAbsoluteEncoderRad()));
        }
       
        else{
            return swerveModuleSim.getSwerveModuleSimPosition();
        }
    }
    

    public SwerveModuleState getState() {
        //creates new swerveModuleState based on drive speed and turn motor position (speed and direction)
        if (Robot.isReal()){
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        }
        else{
            return swerveModuleSim.getSimModuleState();
        }
    }


    public void setDesiredState(SwerveModuleState state) {
        var encoderRotation = new Rotation2d(getTurningPosition());
        // Stops the motors if the desired state is too small
        if (Math.abs(state.speedMetersPerSecond) < 0.001 && !SwerveS.autoLock) {
            stop();
            return;
        }

        // Optimizing finds the shortest path to the desired angle
        state = SwerveModuleState.optimize(state, getState().angle);

        //scales DOWN movement perpendicular to desired direction that occurs when modules change directions. makes smoother.
        //basically, when NOT facing the right direction, turn down our speed so we dont do weird S curves.
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos(); //confirm good idea

        // TODO: Run sysID on these, and get feedforwards for both
        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
        drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

        final double driveFeedforward = driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            turningPIDController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());

        final double turnFeedforward =
            turningFeedForward.calculate(turningPIDController.getSetpoint().velocity);

            if (Robot.isReal()){
                driveMotor.setVoltage(driveOutput + driveFeedforward);
                turningMotor.setVoltage(turnOutput + turnFeedforward);
            }
            else{
                swerveModuleSim.updateVoltage((driveOutput + driveFeedforward), (turnOutput + turnFeedforward));
            }

    }
    public double getDrivePosRadiansSim(){
        return swerveModuleSim.getDrivePosRadiansSim();
    }

    public double getTurningPosRadiansSim(){
        return swerveModuleSim.getTurningPosRadiansSim();
    }

    public double getDriveVelocityMetersPerSecond(){
        return swerveModuleSim.getDriveVelocityMetersPerSecond();
    }
    
    public double getTurningVelocityMetersPerSecond(){
        return swerveModuleSim.getTurningVelocityMetersPerSecond();
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    public void updateSimModuleState(){
        swerveModuleSim.updateModuleState();
    }
    

}
