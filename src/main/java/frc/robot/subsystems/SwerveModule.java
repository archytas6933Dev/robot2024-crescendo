package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Drive;


public class SwerveModule {
    private final CANSparkMax driveMotor;  
    private final CANSparkMax turningMotor;
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(
        int driveMotorId,
        int turningMotorId,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        int absoluteEncoderID,
        double absoluteEncoderOffset,
        boolean absoluteEncoderReversed) {
            
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderID);

        CANCoderConfiguration config = new CANCoderConfiguration();
        // set units of the CANCoder to radians, with velocity being radians per second

        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.sensorDirection = true;
        absoluteEncoder.configAllSettings(config);

        driveMotor = new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);

        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Swerve.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Swerve.kDriveEncoderRPM2MeterPerSec);

        turningEncoder.setPositionConversionFactor(Swerve.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(Swerve.kTurningEncoderRPM2RadPerSec);
        
        turningPidController = new PIDController(Swerve.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.setSmartCurrentLimit(Drive.currentlimit);

        resetEncoder();
    }

    public double getDrivePosition()
    {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition()
    {
        return absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad;
    }

    public double getDriveVelocity()
    {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity()
    {
        return absoluteEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad()
    {
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed? 1.0 : -1.0);
    }

    public void resetEncoder()
    {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
        //absoluteEncoder.setPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        //deadband
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Drive.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
    }

    public void stop()
    {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}

