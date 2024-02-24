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

public class AutoDriveCommand extends Command {
  /** Creates a new AutoDriveCommand. */
  private double xSpeed, ySpeed, distance, facing;
  private Pose2d position;
  private SwerveSubsystem swerveSubsystem;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /*
   *  RED
  *      90
  *  180      0
  *     270
   *  DRIVER
   * 
   * 
   */


  public AutoDriveCommand(SwerveSubsystem swerveSubsystem, double angle, double velocity, double distance, double facing) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.distance = distance;
    this.facing = facing;
    this.xSpeed = Math.cos(Math.toRadians(angle))*velocity;
    this.ySpeed = Math.sin(Math.toRadians(angle))*velocity;
    this.xLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.position = swerveSubsystem.getPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed = 0;
    double currentRotation = swerveSubsystem.getPos().getRotation().getDegrees() % 360;
    double angle = ((180 + (facing - currentRotation)) % 360)-180;

    if(Math.abs(angle)>2.5){
      turnSpeed = angle/180;
    }

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

    return position.getTranslation().getDistance(swerveSubsystem.getPos().getTranslation())>=distance;
  }
}
