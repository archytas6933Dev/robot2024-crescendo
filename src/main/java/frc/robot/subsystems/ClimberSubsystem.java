// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private WPI_TalonSRX motor= new WPI_TalonSRX(Climber.MOTOR_ID);
  public ClimberSubsystem() {
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyClosed);   
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyClosed);

  }



  public void set(double output){
    motor.set(ControlMode.PercentOutput, output);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
