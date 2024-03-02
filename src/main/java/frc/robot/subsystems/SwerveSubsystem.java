package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;

public class SwerveSubsystem extends SubsystemBase
{
    private final SensorSubsystem sensorsubsystem;
    private boolean hasSetHeading = false;
    // private SwerveDrivePoseEstimator positionEstimator;

    private final SwerveModule frontLeft = new SwerveModule(
            Drive.kFrontLeftDriveMotorPort,
            Drive.kFrontLeftTurningMotorPort,
            Drive.kFrontLeftDriveEncoderReversed,
            Drive.kFrontLeftTurningEncoderReversed,
            Drive.kFrontLeftDriveAbsoluteEncoderPort,
            Drive.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Drive.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            Drive.kFrontRightDriveMotorPort,
            Drive.kFrontRightTurningMotorPort,
            Drive.kFrontRightDriveEncoderReversed,
            Drive.kFrontRightTurningEncoderReversed,
            Drive.kFrontRightDriveAbsoluteEncoderPort,
            Drive.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Drive.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            Drive.kBackLeftDriveMotorPort,
            Drive.kBackLeftTurningMotorPort,
            Drive.kBackLeftDriveEncoderReversed,
            Drive.kBackLeftTurningEncoderReversed,
            Drive.kBackLeftDriveAbsoluteEncoderPort,
            Drive.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Drive.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            Drive.kBackRightDriveMotorPort,
            Drive.kBackRightTurningMotorPort,
            Drive.kBackRightDriveEncoderReversed,
            Drive.kBackRightTurningEncoderReversed,
            Drive.kBackRightDriveAbsoluteEncoderPort,
            Drive.kBackRightDriveAbsoluteEncoderOffsetRad,
            Drive.kBackRightDriveAbsoluteEncoderReversed);
    
    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Drive.kDriveKinematics, 
            // getRotation2d(),  new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});

    private SwerveDrivePoseEstimator poseEstimator;
    public SwerveSubsystem(SensorSubsystem sensorsubsystem)
    {
        this.sensorsubsystem = sensorsubsystem;
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                System.out.println("Zeroing Heading");
                zeroHeading();
                poseEstimator = new SwerveDrivePoseEstimator(Drive.kDriveKinematics, 
            getRotation2d(),  new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},new Pose2d());
                sensorsubsystem.setTargetRotation(getHeading());
            } catch (Exception e){
            }
        }).start();
    }

    public void zeroHeading() {
        sensorsubsystem.resetheading();
    }

    public double getHeading(){
        // return 0;
        if(sensorsubsystem==null){
            return 0;
        }
        return Math.IEEEremainder(sensorsubsystem.getheading(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPos() {
        // return odometer.getPoseMeters();
        return poseEstimator.getEstimatedPosition();

    }

    public void resetOdometry(Pose2d pose){
        // odometer.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},pose);
        poseEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},pose);

    }

    @Override
    public void periodic() 
    {
        if(poseEstimator !=null){
            poseEstimator.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
            if(sensorsubsystem.canSeeTags()){
                poseEstimator.addVisionMeasurement(sensorsubsystem.getPosition(),Timer.getFPGATimestamp());
                if(!hasSetHeading){
                    sensorsubsystem.setTargetRotation(sensorsubsystem.getPosition().getRotation().getDegrees());
                    hasSetHeading = true;
                }
            }
            Pose2d position = poseEstimator.getEstimatedPosition();
            SmartDashboard.putNumber("X", position.getX());
            SmartDashboard.putNumber("Y", position.getY());
            SmartDashboard.putNumber("Heading", position.getRotation().getDegrees());
            
        }
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drive.kPhysicalMaxSpeedMetersPerSecond);
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
