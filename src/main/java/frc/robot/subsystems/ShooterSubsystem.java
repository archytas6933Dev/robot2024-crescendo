// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//tihis is a git test
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ShooterSubsystem extends SubsystemBase {

  // TalonFX motor_ = new TalonFX(1); 
  // CANSparkMax motor_ = new CanSparkMax(1);
  // WPI_TalonSRX motor_ = new WPI_TalonSRX(11);
  // WPI_VictorSPX follower_ = new WPI_VictorSPX(13);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() 
  {
    // follower_.follow(motor_);
  }


  public void setshotspeed(double speed)
  {
    // motor_.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
