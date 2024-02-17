// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Intake;
import frc.robot.Constants;
import frc.robot.Constants.Control;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class SwerveJoystickCmd extends Command 
{

  private final SwerveSubsystem swerveSubsystem;
  private final SensorSubsystem sensorSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;


  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Joystick driverJoystick, operatorJoystick;

  // private double shotSpeed;
  private boolean isShooting = false;
  private boolean autoMode = false;
  private long shotTimer = -1;
  
  public SwerveJoystickCmd(
    Joystick driverJoystick, 
    Joystick operatorJoystick,
    SwerveSubsystem swerveSubsystem, 
    SensorSubsystem sensorSubsystem,
    ShooterSubsystem shootersubsystem,
    IntakeSubsystem intakeSubsystem)
  
{
    this.driverJoystick = driverJoystick;   
    this.operatorJoystick = operatorJoystick;   

    this.intakeSubsystem = intakeSubsystem;
    this.sensorSubsystem = sensorSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.shooterSubsystem = shootersubsystem;
    this.xLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shot Speed", 7000);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //get condition of controls
    if(operatorJoystick.getRawButton(Control.ABUTTON)){
      autoMode = true;
    }
    if(operatorJoystick.getRawButton(Control.BBUTTON)){
      autoMode = false;
    }

    boolean isAutoShoot = false;
    boolean isAutoIntake = false;
    if(autoMode && sensorSubsystem.isNoteClose() && !intakeSubsystem.hasNote() && !isShooting){
      isAutoIntake = true;
    }
    if(autoMode && sensorSubsystem.canSeeTags() && sensorSubsystem.isTargetClose() && intakeSubsystem.isShotReady()){
      isAutoShoot = true;
    }
    if(shotTimer>0){
      isAutoShoot = true;
    }

    double xSpeed = driverJoystick.getRawAxis(Control.LEFT_X_AXIS);
    double ySpeed = -driverJoystick.getRawAxis(Control.LEFT_Y_AXIS);
    double xTurn = driverJoystick.getRawAxis(Control.RIGHT_X_AXIS);
    double yTurn = driverJoystick.getRawAxis(Control.RIGHT_Y_AXIS);
    boolean isFieldCentric = !driverJoystick.getRawButton(Control.LBBUTTON);

    // shotSpeed = SmartDashboard.getNumber("Shot Speed", 0);
    
    double intakeAxis = operatorJoystick.getRawAxis(Control.LEFT_Y_AXIS);
    double shooterAxis = operatorJoystick.getRawAxis(Control.LEFT_TRIGGER);
    // double feedAxis = operatorJoystick.getRawAxis(Control.RIGHT_TRIGGER);

    double intakeSpeed = 0;
    double shotSpeed = 0;

      if(intakeAxis<-0.5 && !intakeSubsystem.isShotReady()){
        intakeSpeed=Constants.Intake.INTAKE_SPEED;
      }
      
      // else if(intakeAxis>0.5){
      //   intakeSpeed=Constants.Intake.SPIT_SPEED;
      // }
      // else if(feedAxis>0.5){
      //   intakeSubsystem.setIntakeSpeed(Intake.FEED_SPEED);
      // }
      if(shooterAxis>0.5){
        shotSpeed = Constants.Shooter.SHOT_MEDIUM;
        if((shooterSubsystem.isReady() && intakeSubsystem.isShotReady()) || isShooting){
          intakeSpeed = Intake.FEED_SPEED;
          isShooting = true;
        }
      }
      else{
        isShooting = false;
      }


    double targetRotation = Math.toDegrees(Math.atan2(-xTurn,yTurn)) % 360;
    double turnSpeed = 0;
    
    double currentRotation = swerveSubsystem.getPos().getRotation().getDegrees() % 360;
    double angle = ((360 + (targetRotation - currentRotation)) % 360)-180;

    if(Math.abs(xTurn) + Math.abs(yTurn) > 0.5) {
      turnSpeed = angle/180.0;
    }
    if(!isFieldCentric){
      turnSpeed = xTurn;
    }


    xSpeed = Math.abs(xSpeed) > Control.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > Control.kDeadband ? turnSpeed : 0.0;
   
    //precision mode
    if(driverJoystick.getRawButton(Control.RBBUTTON)){
      xSpeed /= 4;
      ySpeed /= 4;
      turnSpeed /= 4;
    }
    //target tag


    int pov = driverJoystick.getPOV();

    if(isAutoIntake){
      isFieldCentric = false;
      xSpeed = sensorSubsystem.noteTargetX / 60;
      ySpeed = 0.5;
      xSpeed = Math.abs(xSpeed) > Control.kDeadband ? xSpeed : 0.0;
      intakeSpeed = intakeSubsystem.isShotReady()?0:Intake.INTAKE_SPEED;

      // ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
      // turnSpeed = sensorSubsystem.targetRot / 900;
      // if ((sensorSubsystem.currentX == 0) && (sensorSubsystem.currentY == 0)) {
      //   xSpeed = turnSpeed > 0 ? -0.3 : 0.3; // if invisible, slide left/right logically
      // }
    }
    if(isAutoShoot){
      isFieldCentric = false;
      xSpeed = -sensorSubsystem.shotTargetX / 20;
      ySpeed = sensorSubsystem.shotTargetY / 20;
      xSpeed = Math.abs(xSpeed) > Control.kDeadband ? xSpeed : 0.0;
      ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
      shotSpeed=Constants.Shooter.SHOT_MEDIUM;

      if((xSpeed == 0 && ySpeed==0 && shooterSubsystem.isReady()) || shotTimer>0){
        intakeSpeed = Intake.FEED_SPEED;
        isShooting = true;
        if(shotTimer == -1){
          shotTimer = System.currentTimeMillis();
        }
        else if(System.currentTimeMillis()-shotTimer>Constants.Shooter.SHOTTOTALTIME){
          shotTimer = -1;
          isShooting = false;
          shotSpeed = 0;
        }
      }

      // ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
      // turnSpeed = sensorSubsystem.targetRot / 900;
      // if ((sensorSubsystem.currentX == 0) && (sensorSubsystem.currentY == 0)) {
      //   xSpeed = turnSpeed > 0 ? -0.3 : 0.3; // if invisible, slide left/right logically
      // }
    }
    // else if (pov == Control.DAXISS) {
    //   // turnSpeed = sensorSubsystem.targetX/80;    
    //   // turnSpeed = Math.abs(turnSpeed) > Control.kDeadband ? turnSpeed : 0.0;
    // }
    xSpeed = xLimiter.calculate(xSpeed) * Drive.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * Drive.kTeleDriveMaxSpeedMetersPerSecond;
    turnSpeed = turningLimiter.calculate(turnSpeed) * Drive.kTeleDriveMaxAngularSpeedRadiansPerSecond;


    ChassisSpeeds chassisSpeeds;
    if (isFieldCentric)
    {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turnSpeed, swerveSubsystem.getPos().getRotation());
    }
    else
    {
      chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turnSpeed);
    }
    // if((sensorSubsystem.obstacle1 < 36) && (chassisSpeeds.vyMetersPerSecond>0))
    // {
    //   chassisSpeeds.vyMetersPerSecond /= 16/(sensorSubsystem.obstacle1-20);
    // }



    SwerveModuleState[] moduleStates = Drive.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
    // SmartDashboard.putNumber("rotation", moduleStates[0].angle.getRadians());

    //Operator


    if(shooterSubsystem!=null){

      shooterSubsystem.setshotspeed(shotSpeed);
    }

    if(intakeSubsystem.isGrabbed() && !intakeSubsystem.isShotReady()){
      intakeSpeed = Intake.INTAKE_SPEED;
    }



    intakeSubsystem.setIntakeSpeed(intakeSpeed);




    //dashboard
    SmartDashboard.putBoolean("auto mode", autoMode);
    SmartDashboard.putBoolean("auto intake", isAutoIntake);
    SmartDashboard.putBoolean("auto shoot", isAutoShoot);


    // SmartDashboard.putNumber("Target Heading", targetRotation);
    // SmartDashboard.putNumber("Heading", currentRotation);

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
