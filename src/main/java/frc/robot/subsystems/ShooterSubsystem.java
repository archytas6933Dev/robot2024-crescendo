// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//differnt messagge
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ShooterSubsystem extends SubsystemBase {

  WPI_TalonFX motor_ = new WPI_TalonFX(1);
  WPI_TalonFX follower_ = new WPI_TalonFX(2);
  double requestedSpeed = 0;

  // CANSparkMax motor_ = new CanSparkMax(1);
  // WPI_TalonSRX motor_ = new WPI_TalonSRX(11);
  // WPI_VictorSPX follower_ = new WPI_VictorSPX(13);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() 
  {
    //shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor_.setSelectedSensorPosition(0);
    //shooter.setSensorPhase(true);
    motor_.config_kF(0, Constants.Shooter.MOTOR_VELOCITY_F);            
    motor_.config_kP(0, Constants.Shooter.MOTOR_VELOCITY_P);
    motor_.config_kI(0, Constants.Shooter.MOTOR_VELOCITY_I);
    motor_.config_kD(0, Constants.Shooter.MOTOR_VELOCITY_D);
    follower_.follow(motor_);
    motor_.setNeutralMode(NeutralMode.Coast);
    follower_.setNeutralMode(NeutralMode.Coast);
  }


  public void setshotspeed(double speed)
  {
    requestedSpeed = speed;
     motor_.set(ControlMode.Velocity, speed);
  }
  public boolean isReady(){
    
    return(motor_.getSelectedSensorVelocity()>=requestedSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", motor_.getSelectedSensorVelocity());
  }
}
