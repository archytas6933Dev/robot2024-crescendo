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
  static DigitalInput intakeSwitch1 = new DigitalInput(Constants.Intake.SWITCH1_ID);
  static DigitalInput intakeSwitch2 = new DigitalInput(Constants.Intake.SWITCH2_ID);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motor_.config_kF(0, Constants.Intake.MOTOR_POSITION_F);            
    motor_.config_kP(0, Constants.Intake.MOTOR_POSITION_P);
    motor_.config_kI(0, Constants.Intake.MOTOR_POSITION_I);
    motor_.config_kD(0, Constants.Intake.MOTOR_POSITION_D);
  }
  public boolean hasNote(){
    return (intakeSwitch1.get() || intakeSwitch2.get());
  }

  public void setIntakeSpeed(double speed){
    if(speed == 0){
      motor_.setSelectedSensorPosition(0);
      motor_.set(ControlMode.Position, 0);
    }
    else{
      motor_.set(ControlMode.PercentOutput, speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putBoolean("Intake", hasNote());
  
  }
}


