// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SwerveJoystickCmd extends Command 
{

  private final SwerveSubsystem swerveSubsystem;
  private final SensorSubsystem sensorSubsystem;
  private final ShooterSubsystem shootersubsystem;

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Joystick driverJoystick;
  
  public SwerveJoystickCmd(
    Joystick driverJoystick, 
    SwerveSubsystem swerveSubsystem, 
    SensorSubsystem sensorSubsystem,
    ShooterSubsystem shootersubsystem)
  
{
    this.driverJoystick = driverJoystick;   
    this.sensorSubsystem = sensorSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.shootersubsystem = shootersubsystem;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // double shotspeed = driverJoystick.getRawAxis(OIConstants.kDriverYAxis);
    // shootersubsystem.setshotspeed(shotspeed);

    double xSpeed = driverJoystick.getRawAxis(OIConstants.kDriverXAxis);
    double ySpeed = -driverJoystick.getRawAxis(OIConstants.kDriverYAxis);
    double xTurn = driverJoystick.getRawAxis(OIConstants.kDriverRotXAxis);
    double yTurn = driverJoystick.getRawAxis(OIConstants.kDriverRotYAxis);
    boolean isFieldCentric = !driverJoystick.getRawButton(OIConstants.LBBUTTON);

    double targetRotation = Math.toDegrees(Math.atan2(-xTurn,yTurn)) % 360;
    double turnSpeed = 0;
    
    double currentRotation = swerveSubsystem.getRotation2d().getDegrees() % 360;
    double angle = ((360 + (targetRotation - currentRotation)) % 360)-180;

    if(Math.abs(xTurn) + Math.abs(yTurn) > 0.5) {
      turnSpeed = angle/180.0;
    }
    if(!isFieldCentric){
      turnSpeed = xTurn;
    }


    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;
   
    //precision mode
    if(driverJoystick.getRawButton(OIConstants.RBBUTTON)){
      xSpeed /= 4;
      ySpeed /= 4;
      turnSpeed /= 4;
    }
    //target tag
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);

    int pov = driverJoystick.getPOV();

    if(pov == OIConstants.DAXISN){
      isFieldCentric = false;
      xSpeed = sensorSubsystem.targetX / 60;
      ySpeed = sensorSubsystem.targetY / 60;
      xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
      ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
      turnSpeed = sensorSubsystem.targetRot / 900;
      if ((sensorSubsystem.currentX == 0) && (sensorSubsystem.currentY == 0)) {
        xSpeed = turnSpeed > 0 ? -0.3 : 0.3; // if invisible, slide left/right logically
      }
    }
    else if (pov == OIConstants.DAXISS) {
      turnSpeed = sensorSubsystem.targetX/80;    
      turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;
    }

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turnSpeed = turningLimiter.calculate(turnSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    if (isFieldCentric)
    {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turnSpeed, swerveSubsystem.getRotation2d());
    }
    else
    {
      chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

    //dashboard
    SmartDashboard.putNumber("Target Heading", targetRotation);
    SmartDashboard.putNumber("Heading", currentRotation);
    // SmartDashboard.putNumber("difference", angle);
    // SmartDashboard.putNumber("turnSpeed", turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
