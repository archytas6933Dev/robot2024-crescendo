package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase
{
    private final SensorSubsystem sensorsubsystem;

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
            getRotation2d(),  new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});

    public SwerveSubsystem(SensorSubsystem sensorsubsystem)
    {
        this.sensorsubsystem = sensorsubsystem;
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){
            }
        }).start();
    }

    public void zeroHeading() {
        // sensorsubsystem.resetheading();
    }

    public double getHeading(){
        return 0;
        // return Math.IEEEremainder(sensorsubsystem.getheading(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPos() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},pose);
    }

    @Override
    public void periodic() 
    {
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    
    }

    public void updateDashboard()
    {
        // SmartDashboard.putNumber("Encoder 1", frontRight.getTurningPosition());
        // SmartDashboard.putNumber("Encoder 2", frontLeft.getTurningPosition());
        // SmartDashboard.putNumber("Encoder 3", backLeft.getTurningPosition());
        // SmartDashboard.putNumber("Encoder 4", backRight.getTurningPosition());
        // SmartDashboard.putNumber("Rad 1", frontRight.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Rad 2", frontLeft.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Rad 3", backLeft.getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Rad 4", backRight.getAbsoluteEncoderRad());
    }
}
