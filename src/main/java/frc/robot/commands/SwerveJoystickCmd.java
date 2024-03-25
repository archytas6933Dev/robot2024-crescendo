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
import frc.robot.Constants.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Control;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class SwerveJoystickCmd extends Command 
{

  private final SwerveSubsystem swerveSubsystem;
  private final SensorSubsystem sensorSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem;


  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Joystick driverJoystick, operatorJoystick;

  // private double shotSpeed;
  private boolean isShooting = false;
  private boolean autoMode = false;
  private long shotTimer = -1;
  private long intakeTimer = -1;
  private boolean isClimbing = false;
  
  public SwerveJoystickCmd(
    Joystick driverJoystick, 
    Joystick operatorJoystick,
    SwerveSubsystem swerveSubsystem, 
    SensorSubsystem sensorSubsystem,
    ShooterSubsystem shootersubsystem,
    IntakeSubsystem intakeSubsystem,
    ClimberSubsystem climberSubsystem)
  
{
    this.driverJoystick = driverJoystick;   
    this.operatorJoystick = operatorJoystick;   

    this.intakeSubsystem = intakeSubsystem;
    this.sensorSubsystem = sensorSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.shooterSubsystem = shootersubsystem;
    this.climberSubsystem = climberSubsystem;
    this.xLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("Shot Speed", 7000);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //get condition of controls
    if(operatorJoystick.getRawButton(Control.ABUTTON)){
      autoMode = true;
    }
    if(driverJoystick.getRawButton(Control.ABUTTON)){
      autoMode = true;
    }
    if(operatorJoystick.getRawButton(Control.BBUTTON)){
      autoMode = false;
    }
    if(driverJoystick.getRawButton(Control.BBUTTON)){
      autoMode = false;
    }

    boolean isAutoShoot = false;
    boolean isAutoIntake = false;
    if(autoMode && sensorSubsystem.isNoteClose() && !intakeSubsystem.hasNote() && !isShooting){
      isAutoIntake = true;
    }
    if(autoMode && sensorSubsystem.isTargetClose() && intakeSubsystem.isShotReady() 
    // && (driverJoystick.getRawAxis(Control.RIGHT_TRIGGER)>0.5)
    ){
      isAutoShoot = true;
    }
    if(shotTimer>0){
      isAutoShoot = true;
    }
    double inversion = sensorSubsystem.isRed?1:-1;
    double xSpeed = driverJoystick.getRawAxis(Control.LEFT_X_AXIS) * inversion;
    double ySpeed = -driverJoystick.getRawAxis(Control.LEFT_Y_AXIS) * inversion;
    double xTurn = driverJoystick.getRawAxis(Control.RIGHT_X_AXIS) * inversion;
    double yTurn = driverJoystick.getRawAxis(Control.RIGHT_Y_AXIS) * inversion;
    boolean isTrigger = driverJoystick.getRawAxis(Control.LEFT_TRIGGER)>0.5;
    boolean isFieldCentric = !isTrigger;

    // shotSpeed = SmartDashboard.getNumber("Shot Speed", 0);
    
    double intakeAxis = operatorJoystick.getRawAxis(Control.LEFT_Y_AXIS);
    double shooterAxis = operatorJoystick.getRawAxis(Control.LEFT_TRIGGER);
    double climbAxis = operatorJoystick.getRawAxis(Control.RIGHT_Y_AXIS);
    int tiltAxis = operatorJoystick.getPOV();
    // double feedAxis = operatorJoystick.getRawAxis(Control.RIGHT_TRIGGER);
    if(climberSubsystem!=null){
      if(climbAxis>0.5){
        isClimbing = true;
        shooterSubsystem.setTiltPosition(0);
        climberSubsystem.set(Climber.SPEED);
      }
      else if(climbAxis<-0.5){
        shooterSubsystem.setTiltPosition(Shooter.TILT_CLIMB);
        climberSubsystem.set(-Climber.SPEED);
      }
      else{
        climberSubsystem.set(0);
      }
    }

    if(operatorJoystick.getRawButton(Control.RIGHTJOYCLICK) || driverJoystick.getRawButton(Control.XBUTTON)){
      isClimbing = false;
    }

    if(tiltAxis==Constants.Control.DAXISN){
      shooterSubsystem.setTiltPosition(Shooter.TILT_HIGH);
      sensorSubsystem.setTargetYOffset(Shooter.TILT_HIGH);
    }
    if(tiltAxis==Constants.Control.DAXISE || tiltAxis == Control.DAXISW){
      shooterSubsystem.setTiltPosition(Shooter.TILT_MEDIUM);
      sensorSubsystem.setTargetYOffset(Shooter.TILT_MEDIUM);

    }
    if(tiltAxis==Constants.Control.DAXISS){
      shooterSubsystem.setTiltPosition(Shooter.TILT_LOW);
      sensorSubsystem.setTargetYOffset(Shooter.TILT_LOW);

    } 
    if(operatorJoystick.getRawButton(Control.XBUTTON)){
      shooterSubsystem.tiltUp();
    }

    double intakeSpeed = 0;
    double shotSpeed = 0;

      if(intakeAxis<-0.5 && !intakeSubsystem.isShotReady()){
        intakeSpeed=Constants.Intake.INTAKE_SPEED;
      }
      else if(intakeAxis>0.5){
        intakeSpeed=Constants.Intake.SPIT_SPEED;
      }
      // else if(feedAxis>0.5){
      //   intakeSubsystem.setIntakeSpeed(Intake.FEED_SPEED);
      // }

      // SmartDashboard.getNumber("Shot Speed", shotSpeed);
      if(shooterAxis>0.5){
        // shotSpeed = SmartDashboard.getNumber("Shot Speed", shotSpeed);
        shotSpeed = Constants.Shooter.SHOT_MEDIUM;
        if((shooterSubsystem.isReady() && intakeSubsystem.isShotReady()) || isShooting){
          intakeSpeed = Intake.FEED_SPEED;
          isShooting = true;
        }
      }
      else{
        isShooting = false;
      }


    // sensorSubsystem.setTargetRotation(Math.toDegrees(Math.atan2(-xTurn,yTurn)) % 360);
    double turnSpeed = 0;

    if((Math.abs(xTurn) > 0.8) && (Math.abs(yTurn) > 0.8) && !isAutoIntake && !isAutoShoot && isFieldCentric && !isClimbing) {
      if(intakeSubsystem.hasNote()){
        xTurn = -xTurn;
        yTurn = -yTurn;
      }
      sensorSubsystem.setTargetRotation(Math.toDegrees(Math.atan2(xTurn,-yTurn)) % 360);
    
    }

    
    turnSpeed = swerveSubsystem.turnForAngle(sensorSubsystem.getTargetRotation());


    if(driverJoystick.getRawButton(Control.RBBUTTON) && !isClimbing){
      turnSpeed = 0.5;
    }

    if(driverJoystick.getRawButton(Control.LBBUTTON) && !isClimbing){
      turnSpeed = -0.5;
    }


    if(!isFieldCentric && !isAutoIntake && !isAutoShoot){
      double currentRotation = swerveSubsystem.getPos().getRotation().getDegrees() % 360 - 180;
      sensorSubsystem.setTargetRotation(currentRotation);
      turnSpeed = xTurn;
      turnSpeed = Math.abs(turnSpeed) > Control.kDeadband ? turnSpeed : 0.0;
    }

    xSpeed = Math.abs(xSpeed) > Control.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
   
    //precision mode
    if(isTrigger || isClimbing){
      xSpeed /= 4;
      ySpeed /= 4;
      turnSpeed /= 4;
    }
    //target tag


    int pov = driverJoystick.getPOV();

    if(isAutoIntake){
      isFieldCentric = false;
      xSpeed = sensorSubsystem.noteTargetX / 60;
      ySpeed = 0.8;
      xSpeed = Math.abs(xSpeed) > Control.kDeadband ? xSpeed : 0.0;
      intakeSpeed = intakeSubsystem.isShotReady()?0:Intake.INTAKE_SPEED;
      intakeTimer = sensorSubsystem.getTime();

      // ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
      // turnSpeed = sensorSubsystem.targetRot / 900;
      // if ((sensorSubsystem.currentX == 0) && (sensorSubsystem.currentY == 0)) {
      //   xSpeed = turnSpeed > 0 ? -0.3 : 0.3; // if invisible, slide left/right logically
      // }
    }
    if( autoMode && sensorSubsystem.getTime()-intakeTimer<Constants.Intake.INTAKE_STALE && intakeTimer!=-1 && intakeSubsystem.isShotReady()){
      sensorSubsystem.setTargetRotation(sensorSubsystem.isRed?0:180);
      intakeTimer = -1;
    }
    if(isAutoShoot){
      isFieldCentric = false;
      xSpeed = -sensorSubsystem.shotTargetX / 30;
      ySpeed = sensorSubsystem.shotTargetY / 40;
      // xSpeed = Math.abs(xSpeed) > Control.kDeadband ? xSpeed : 0.0;
      // ySpeed = Math.abs(ySpeed) > Control.kDeadband ? ySpeed : 0.0;
      shotSpeed=Constants.Shooter.SHOT_MEDIUM;
      double YDeadband = Constants.Shooter.YDeadband;
      double XDeadband = Constants.Shooter.XDeadband;
      if(shooterSubsystem.getTargetTilt() == Constants.Shooter.TILT_HIGH){
        XDeadband *=4;
        YDeadband *=3;
      }

      if(( Math.abs(sensorSubsystem.shotTargetX) <= XDeadband 
          && Math.abs(sensorSubsystem.shotTargetY) <= YDeadband 
          && shooterSubsystem.isReady()) || shotTimer>0)
      {
        intakeSpeed = Intake.FEED_SPEED;
        xSpeed=0;
        ySpeed=0;
        isShooting = true;
        if(shotTimer == -1){
          shotTimer = sensorSubsystem.getTime();
        }
        else if(sensorSubsystem.getTime()-shotTimer>Constants.Shooter.SHOTTOTALTIME){
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
    if(autoMode && sensorSubsystem.isTargetClose() && intakeSubsystem.hasNote()){
      shotSpeed=Constants.Shooter.SHOT_MEDIUM;
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

    //SmartDashboard.putNumber("angle", angle);


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
