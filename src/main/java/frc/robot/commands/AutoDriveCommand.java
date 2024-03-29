// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SensorSubsystem;

public class AutoDriveCommand extends Command {
  /** Creates a new AutoDriveCommand. */
  private double xSpeed, ySpeed, distance, facing;
  private Pose2d position;
  private SwerveSubsystem swerveSubsystem;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private SensorSubsystem sensorSubsystem;
  /*
   *  RED
  *      90
  *  180      0
  *     270
   *  DRIVER
   * 
   * 
   */

  public AutoDriveCommand(SwerveSubsystem swerveSubsystem, SensorSubsystem sensorSubsystem, double angle, double velocity, double distance, double facing) {
    this(swerveSubsystem, sensorSubsystem, angle, velocity, distance, facing, 0, 0);
  }
  public AutoDriveCommand(SwerveSubsystem swerveSubsystem, SensorSubsystem sensorSubsystem, double angle, double velocity, double distance, double facing, double destX, double destY) {
    // Use addRequirements() here to declare subsystem dependencies.S
    this.swerveSubsystem = swerveSubsystem;
    this.sensorSubsystem = sensorSubsystem;
    this.distance = distance;
    this.facing = facing;
    this.xSpeed = Math.cos(Math.toRadians(angle))*velocity;
    this.ySpeed = Math.sin(Math.toRadians(angle))*velocity;
    if(destX !=0 || destY !=0){
      Pose2d pose = swerveSubsystem.getPos();
      double currX = pose.getX();
      double currY = pose.getY();
      double diffX = destX-currX;
      double diffY = destY-currY;
      this.distance = Math.sqrt(diffX*diffX+diffY*diffY);
      double divisor = (Math.abs(diffX)>Math.abs(diffY))? Math.abs(diffX/velocity): Math.abs(diffY/velocity);
        this.xSpeed = diffX/divisor;
        this.ySpeed = diffY/divisor; 
    }
    this.xLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("drive");
    this.position = swerveSubsystem.getPos();
    sensorSubsystem.setTargetRotation(facing);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   double turnSpeed = swerveSubsystem.turnForAngle(facing);


    double tempXSpeed = xLimiter.calculate(xSpeed) * Drive.kTeleDriveMaxSpeedMetersPerSecond;
    double tempYSpeed = yLimiter.calculate(ySpeed) * Drive.kTeleDriveMaxSpeedMetersPerSecond;
    double tempTurnSpeed = turningLimiter.calculate(turnSpeed) * Drive.kTeleDriveMaxAngularSpeedRadiansPerSecond;


    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(tempYSpeed, tempXSpeed, tempTurnSpeed, swerveSubsystem.getPos().getRotation());


    SwerveModuleState[] moduleStates = Drive.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerveSubsystem.getPos().getRotation());


    SwerveModuleState[] moduleStates = Drive.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

//    if (sensorSubsystem.isTargetClose() && intakeSubsystem.isShotReady())

    return position.getTranslation().getDistance(swerveSubsystem.getPos().getTranslation())>=distance;
  }
}
