// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{ 
  WPI_TalonFX motor_ = new WPI_TalonFX(Constants.Intake.MOTOR_ID);
  static DigitalInput intakeSwitch = new DigitalInput(Constants.Intake.SWITCH1_ID);
  static DigitalInput feedSwitch = new DigitalInput(Constants.Intake.SWITCH2_ID);
  private double requestedSpeed;

  private long curTime = 0;
  public long lastSawIntake = 0;
  public long lastShotReady = 0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor_.config_kF(0, Constants.Intake.MOTOR_POSITION_F);            
    motor_.config_kP(0, Constants.Intake.MOTOR_POSITION_P);
    motor_.config_kI(0, Constants.Intake.MOTOR_POSITION_I);
    motor_.config_kD(0, Constants.Intake.MOTOR_POSITION_D);

    motor_.config_kF(1, Constants.Intake.MOTOR_VELOCITY_F);            
    motor_.config_kP(1, Constants.Intake.MOTOR_VELOCITY_P);
    motor_.config_kI(1, Constants.Intake.MOTOR_VELOCITY_I);
    motor_.config_kD(1, Constants.Intake.MOTOR_VELOCITY_D);
  }
  public boolean hasNote(){
    return isGrabbed() || isShotReady();
  }
  public boolean isShotReady(){
    if(feedSwitch.get()){
      lastShotReady  = curTime;
    }
    if(lastShotReady +Constants.Intake.SHOT_STALE<curTime){
      return false;
    }
    return true;
    // return feedSwitch.get();
  }
  public boolean isGrabbed(){
    if(intakeSwitch.get()){
      lastSawIntake  =curTime;
    }
    if(lastSawIntake+Constants.Intake.GRABBED_STALE<curTime){
      return false;
    }
    return true;
  }

  public void setIntakeSpeed(double speed){
    if(speed == requestedSpeed){
      return;
    }

    if(speed == 0){
      motor_.setSelectedSensorPosition(0);
      motor_.selectProfileSlot(0, 0);
      motor_.set(ControlMode.Position, 0);
    }
    else{
      motor_.selectProfileSlot(1, 0);
      motor_.set(ControlMode.Velocity, speed);
    }
    requestedSpeed = speed;
  }

  @Override
  public void periodic() {
    curTime = System.currentTimeMillis();
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake", isGrabbed());
    SmartDashboard.putBoolean("ShotReady", isShotReady());
    //System.out.println(isShotReady() + ", " + feedSwitch.get());

  }
}


