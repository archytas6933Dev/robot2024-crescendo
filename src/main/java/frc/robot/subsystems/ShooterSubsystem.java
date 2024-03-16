// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//differnt messagge
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ShooterSubsystem extends SubsystemBase {

  WPI_TalonFX left_ = new WPI_TalonFX(41);
  WPI_TalonFX right_ = new WPI_TalonFX(42);
  WPI_TalonFX tilt_ = new WPI_TalonFX(61);
  // private PIDController pidController = new PIDController(1/7000, 0, 0);
  //  private final SimpleMotorFeedforward m_shooterFeedforward =
  //     new SimpleMotorFeedforward(
  //         Constants.Shooter.kSVolts, Constants.Shooter.kVVoltSecondsPerRotation);
  double requestedSpeed = 0;
  double targetPosition = Shooter.TILT_LOW;

  // CANSparkMax motor_ = new CanSparkMax(1);
  // WPI_TalonSRX motor_ = new WPI_TalonSRX(11);
  // WPI_VictorSPX follower_ = new WPI_VictorSPX(13);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() 
  {

    //shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    left_.setSelectedSensorPosition(0);
    //shooter.setSensorPhase(true);
    left_.config_kF(0, Constants.Shooter.MOTOR_VELOCITY_F);            
    left_.config_kP(0, Constants.Shooter.MOTOR_VELOCITY_P);
    left_.config_kI(0, Constants.Shooter.MOTOR_VELOCITY_I);
    left_.config_kD(0, Constants.Shooter.MOTOR_VELOCITY_D);

    right_.config_kF(0, Constants.Shooter.MOTOR_VELOCITY_F);            
    right_.config_kP(0, Constants.Shooter.MOTOR_VELOCITY_P);
    right_.config_kI(0, Constants.Shooter.MOTOR_VELOCITY_I);
    right_.config_kD(0, Constants.Shooter.MOTOR_VELOCITY_D);

    left_.setNeutralMode(NeutralMode.Coast);
    right_.setNeutralMode(NeutralMode.Coast);
    left_.configSelectedFeedbackCoefficient(1);
    right_.configSelectedFeedbackCoefficient(1);

    tilt_.config_kF(0, Constants.Shooter.TILT_POSITION_F);            
    tilt_.config_kP(0, Constants.Shooter.TILT_POSITION_P);
    tilt_.config_kI(0, Constants.Shooter.TILT_POSITION_I);
    tilt_.config_kD(0, Constants.Shooter.TILT_POSITION_D);
    tilt_.setSelectedSensorPosition(0);
  }

  public double getTiltPosition(){
    return tilt_.getSelectedSensorPosition();
  }

  public double getTiltSpeed(){
    return Math.abs(tilt_.getSelectedSensorVelocity());
  }

  public double getTargetTilt(){
    return targetPosition;
  }

  public void setTiltPosition(double position){
    targetPosition = position;
    tilt_.set(ControlMode.Position, position);
  }

  public void setshotspeed(double speed)
  {
    requestedSpeed = speed;
    // left_.set(ControlMode.PercentOutput, -speed/200);
    // right_.set(ControlMode.PercentOutput, speed/100);

    // left_.getSelectedSensorVelocity();
    if(Math.abs(speed) == 0){
      left_.set(ControlMode.Velocity, 0);
      right_.set(ControlMode.Velocity, 0);

      left_.set(ControlMode.PercentOutput, 0);
      right_.set(ControlMode.PercentOutput, 0);

    }
    else{


      // double output = pidController.calculate(left_.getSelectedSensorVelocity(),speed);
      // left_.set(ControlMode.PercentOutput, output);
      // SmartDashboard.putNumber("calculated",output);
    // }

    left_.set(ControlMode.Velocity, requestedSpeed);
    right_.set(ControlMode.Velocity, -requestedSpeed*.7);
    }

  }
  public boolean isReady(){
    
    return(Math.abs(left_.getSelectedSensorVelocity()-requestedSpeed)<=Shooter.SHOT_TOLERANCE) && Math.abs(requestedSpeed)!=0  && getTiltSpeed()<Constants.Shooter.TILT_THRESHOLD;
  }
  public void tiltUp(){
    tilt_.set(ControlMode.PercentOutput, -0.5);
  }

  @Override
  public void periodic() {
    if(tilt_.isRevLimitSwitchClosed() == 0){
      tilt_.setSelectedSensorPosition(0);
    }
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("requested speed", requestedSpeed);
    SmartDashboard.putBoolean("Can Shoot", isReady());
    // SmartDashboard.putNumber("Shooter Speed left", left_.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Speed right", right_.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Tilt position", getTiltPosition());
    SmartDashboard.putNumber("Tilt speed", getTiltSpeed());


  }
}
