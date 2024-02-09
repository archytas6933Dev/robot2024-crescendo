// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.util.JSONPObject;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class SensorSubsystem extends SubsystemBase {

  private AHRS gyro_;

  public double noteTargetX;
  public double noteTargetY;
  public double shotTargetX;
  public double shotTargetY;

  public double currentRobotX;
  public double currentRobotY;
  public double currentRobotRot;

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
    
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("limelight-intake");

    NetworkTableEntry noteX = intakeTable.getEntry("tx");
    NetworkTableEntry noteY = intakeTable.getEntry("ty");
    noteTargetX = noteX.getDouble(0);
    noteTargetY = noteY.getDouble(0);


    NetworkTableEntry shotX = shooterTable.getEntry("tx");
    NetworkTableEntry shotY = shooterTable.getEntry("ty");
    shotTargetX = shotX.getDouble(0);
    shotTargetY = shotY.getDouble(0);

    NetworkTableEntry position = shooterTable.getEntry("botpose");
    double[] stuff = position.getDoubleArray(new double[1]);
    String str = "[";
    for(int i=0;i<stuff.length;i++){
      String a = Double.toString(stuff[i]);
      str+=a.substring(0,Math.min(a.length(),4))+",";
    }
    str=str.substring(0,str.length()-1)+"]";

    NetworkTableEntry jsonStuff = shooterTable.getEntry("json");
    ObjectMapper mapper = new ObjectMapper();
    try {
      JsonNode node = mapper.readTree(jsonStuff.getString(""));
      
    } catch (JsonMappingException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (JsonProcessingException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }



    // currentX = tx.getDouble(0.0);
    // currentY = ty.getDouble(0.0);
    // double[] rotation = rotationEntry.getDoubleArray(new double[0]);
    // if(rotation.length != 0){
    //   currentRot = Math.toDegrees(rotation[0]);
    // }
    // if (currentX != 0) targetX = currentX;
    // if (currentY != 0) targetY = currentY;
    // if (currentRot != 0) targetRot = currentRot;

    // double value = ultrasonic1.getAverageVoltage();
    // obstacle1 = value*12/.3;
    // SmartDashboard.putNumber("Obstacle", obstacle1);
    // double value2 = ultrasonic2.getAverageVoltage();
    // obstacle2 = value2*12/.3;
    // SmartDashboard.putNumber("Obstacle 2", obstacle2);
    
    SmartDashboard.putNumber("Heading", gyro_.getAngle());
    SmartDashboard.putNumber("note X", noteTargetX);
    SmartDashboard.putNumber("note Y", noteTargetY);
    SmartDashboard.putNumber("shot X", shotTargetX);
    SmartDashboard.putNumber("shot Y", shotTargetY);
    SmartDashboard.putString("stuff", str);
    SmartDashboard.putString("json", jsonStuff.getString(""));


  }
}
