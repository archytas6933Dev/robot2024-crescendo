// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class SensorSubsystem extends SubsystemBase {

  private AHRS gyro_;

  public double targetX;
  public double targetY;
  public double targetRot;

  public double currentX;
  public double currentY;
  public double currentRot;

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem() 
  {
    gyro_ = new AHRS(SPI.Port.kMXP);
  }

  public void resetheading()
  {
    gyro_.reset();   
  }

  public double getheading()
  {
    return gyro_.getAngle();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-archy");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry rotationEntry = table.getEntry("botpose_targetspace");

    currentX = tx.getDouble(0.0);
    currentY = ty.getDouble(0.0);
    currentRot = Math.toDegrees(rotationEntry.getDoubleArray(new double[0])[0]);
    if (currentX != 0) targetX = currentX;
    if (currentY != 0) targetY = currentY;
    if (currentRot != 0) targetRot = currentRot;
    

  }
}
