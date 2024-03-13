// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class AutoDriveByShootingCommand extends Command {
  /** Creates a new AutoDriveByShootingCommand. */
  private final SwerveSubsystem swerveSubsystem;
  private final SensorSubsystem sensorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final double shotSpeed, shotPosition;
private SlewRateLimiter xLimiter,yLimiter;

  private long shotTime = -1;
  public AutoDriveByShootingCommand( 
  double shotSpeed,  
  double shotPosition, 
  SwerveSubsystem swerveSubsystem, 
  SensorSubsystem sensorSubsystem,
  IntakeSubsystem intakeSubsystem,
  ShooterSubsystem shooterSubsystem) {
    this.shotSpeed = shotSpeed;
    this.shotPosition = shotPosition;
    this.sensorSubsystem = sensorSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.xLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sensorSubsystem);
    addRequirements(swerveSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("shoot");

    shooterSubsystem.setshotspeed(shotSpeed);
    shooterSubsystem.setTiltPosition(shotPosition);
    sensorSubsystem.setTargetYOffset(shotPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //execute when target can be seen and driven straight towards
  public void execute() {
    // Set drive speed based on april tags for target
    if(sensorSubsystem.isAtLeastOneTag()){


    double xSpeed = -sensorSubsystem.shotTargetX / 30;
    double ySpeed = sensorSubsystem.shotTargetY / 30;

    if(( Math.abs(sensorSubsystem.shotTargetX) <= Constants.Shooter.XDeadband 
    && Math.abs(sensorSubsystem.shotTargetY) <= Constants.Shooter.YDeadband 
    && shooterSubsystem.isReady()) || shotTime>0){
      intakeSubsystem.setIntakeSpeed(Constants.Intake.FEED_SPEED);
      xSpeed = 0;
      ySpeed = 0;
      if(shotTime==-1){
        shotTime = sensorSubsystem.getTime();
      }

    }
    double turnSpeed = swerveSubsystem.turnForAngle(sensorSubsystem.getTargetRotation());
    xSpeed = xLimiter.calculate(xSpeed) * Drive.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * Drive.kTeleDriveMaxSpeedMetersPerSecond;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turnSpeed);

    SwerveModuleState[] moduleStates = Drive.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setshotspeed(0);
    intakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(shotTime != -1 && sensorSubsystem.getTime() > shotTime + Constants.Shooter.SHOTTOTALTIME);
  }
}
