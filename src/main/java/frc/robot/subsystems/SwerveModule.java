package frc.robot.subsystems;



//import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final SparkAnalogSensor absoluteEncoder;
    private final AnalogInput absoluteEncoder; // Use either AnalogInput or CANCoder depending on the absolute encoder
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
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
        //absoluteEncoder = new CANCoder(absoluteEncoderId);


        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        //absoluteEncoder = driveMotor.getAnalog(Mode.kAbsolute);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(Constants.SwerveConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(Constants.SwerveConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(Constants.SwerveConstants.kTurningP, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return getAbsoluteEncoderRad();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        angle += angle <= 0 ? 2*Math.PI : 0;
        angle -= Math.PI; //angle > Math.PI ? 2*Math.PI : 0;
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
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }
    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // Stops the motors if the desired state is too small
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimizing finds the shortest path to the desired angle
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
