// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Control;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Iterator;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.util.JSONPObject;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.DigitalOutput;


public class SensorSubsystem extends SubsystemBase {

  private AHRS gyro_;

  public double noteTargetX;
  public double noteTargetY;
  public double shotTargetX;
  public double shotTargetY;

  public double currentRobotX;
  public double currentRobotY;
  public double currentRobotRot;

  private boolean canSee;
  private boolean isTagsClose;
  private boolean canSeeNote;
  private boolean atLeastOneTarget= false;
  private double targetRotation = 0;
  private long curTime = 0;

  private long lastSawNote = 0;
  private long lastSawTarget = 0;
  public boolean isRed;
  private double targetYOffset = 0;

  private DigitalOutput led1;
  private DigitalOutput led2;

  private AnalogInput eye1;
  private AnalogInput eye2;

  /** Creates a new SensorSubsystem. */
  public SensorSubsystem() 
  {
    gyro_ = new AHRS(SPI.Port.kMXP);
    // eye1 = new AnalogInput(0);
    // eye2 = new AnalogInput(1);

    // led1 = new DigitalOutput(3);
    // led2 = new DigitalOutput(4);
  }


  public void resetheading()
  {
    gyro_.reset();   
  }

  public double getheading()
  {
    return gyro_.getAngle();
  }

  public void setTargetRotation(double rot){
    targetRotation = rot;
  }
  public double getTargetRotation(){
    return targetRotation;
  }

  public boolean canSeeTags(){
    return canSee;
  }
  public boolean tagsClose(){
    return isTagsClose;
  }

  public boolean isAtLeastOneTag(){
    return atLeastOneTarget && shotTargetX != Double.NaN && shotTargetY != Double.NaN;
  }
  public boolean isTargetClose(){
    // return false;
    return isAtLeastOneTag() && shotTargetY>Constants.Shooter.AUTO_SHOT_Y ;

  }

  public boolean isNoteClose(){
    // return false;
    return noteTargetY<Constants.Intake.AUTO_NOTE_Y && canSeeNote;
  }
  public boolean isNoteVisible(){
    return canSeeNote;
  }
  public Pose2d getPosition(){
    return new Pose2d(currentRobotX,currentRobotY, Rotation2d.fromDegrees(currentRobotRot));
  }

  public long getTime(){
    return curTime;
  }

  public void setTargetYOffset(double position){
    if(position == Shooter.TILT_HIGH){
      targetYOffset = Shooter.HIGH_OFFSET;
    }
    else if(position == Shooter.TILT_MEDIUM){
      targetYOffset = Shooter.MEDIUM_OFFSET;
    }
    else if(position == Shooter.TILT_LOW){
      targetYOffset = Shooter.LOW_OFFSET;
    }
    else{
      targetYOffset = 0;
    }
  }

  public void setled1(boolean isOn)
  {
    // led1.set(!isOn);
  }

  public void setled2(boolean isOn)
  {
    // led2.set(!isOn);
  }

  @Override
  public void periodic() 
  {

    // SmartDashboard.putNumber("eye1", eye1.getVoltage());
    // SmartDashboard.putNumber("eye2", eye2.getVoltage());

    curTime = System.currentTimeMillis();
    // This method will be called once per scheduler run
    
    NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("limelight-intake");
    NetworkTable FMSinfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
    isRed = FMSinfo.getEntry("IsRedAlliance").getBoolean(true);

    NetworkTableEntry noteX = intakeTable.getEntry("tx");
    NetworkTableEntry noteY = intakeTable.getEntry("ty");
    double tempNoteTargetX = noteX.getDouble(0);
    double tempNoteTargetY = noteY.getDouble(0);
    if(tempNoteTargetX==0 && tempNoteTargetY==0){
      if(lastSawNote<curTime-Constants.Intake.NOTE_STALE){
        noteTargetX = 0;
        noteTargetY=0;
      }
    }
    else{
      lastSawNote = curTime;
      noteTargetX = tempNoteTargetX;
      noteTargetY = tempNoteTargetY;
    }
    canSeeNote = noteTargetX!=0 || noteTargetY!=0;



    NetworkTableEntry position = shooterTable.getEntry("botpose");
    double[] botpose = position.getDoubleArray(new double[1]);
    String str = "[";
    for(int i=0;i<botpose.length;i++){
      String a = Double.toString(botpose[i]);
      str+=a.substring(0,Math.min(a.length(),4))+",";
    }
    str=str.substring(0,str.length()-1)+"]";

    NetworkTableEntry jsonStuff = shooterTable.getEntry("json");
    ObjectMapper mapper = new ObjectMapper();
    int numMarkers = 0;
    int numTargets = 0;
    atLeastOneTarget = false;
    try {
      JsonNode root = mapper.readTree(jsonStuff.getString(""));
      // JsonNode results = root.path("Results");
      JsonNode tags = root.path("Results").path("Fiducial");

      for(int i=0;i<tags.size();i++){
        int tagNum = tags.get(i).path("fID").asInt();
        if(tags.get(i).path("t6t_cs").get(2).asDouble(100000)<Shooter.CLOSE_TAG_THRESHOLD){
          numMarkers++;

        }
        //speaker
        if((tagNum==4 && isRed) || (tagNum == 7 && !isRed)){
          atLeastOneTarget = true;
          shotTargetX = tags.get(i).path("tx").asDouble();
          shotTargetY = tags.get(i).path("ty").asDouble()+targetYOffset;
          lastSawTarget = curTime;
        }
        //amp
        if(tagNum == 5 || tagNum == 6){

        }
      }


      
      numTargets = tags.size();
    } catch (JsonMappingException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (JsonProcessingException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    canSee = false;
    if(numTargets >= 2){
      canSee = true;
      currentRobotX = -botpose[0];
      currentRobotY = botpose[1];
      currentRobotRot = -botpose[5];
    }
    else if(numTargets==0){
      if(lastSawTarget<curTime-Constants.Shooter.TARGET_STALE){
        shotTargetX = Double.NaN;
        shotTargetY = Double.NaN;
      }

    }

    if(numMarkers>=2){
      isTagsClose = true;
    }
    else{
      isTagsClose = false;
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

    // SmartDashboard.putNumber("Heading", gyro_.getAngle());
    SmartDashboard.putNumber("note X", noteTargetX);
    SmartDashboard.putNumber("note Y", noteTargetY);
    SmartDashboard.putNumber("shot X", shotTargetX);
    SmartDashboard.putNumber("shot Y", shotTargetY);
    SmartDashboard.putBoolean("alliance", isRed);
        SmartDashboard.putBoolean("isNoteVisible", isNoteVisible());
      SmartDashboard.putBoolean("isTagsClose", isTagsClose);
    // SmartDashboard.putNumber("robot X", currentRobotX);
    // SmartDashboard.putNumber("robot Y", currentRobotY);
    // SmartDashboard.putNumber("robot rotation", currentRobotRot);
    SmartDashboard.putNumber("number of markers", numMarkers);
    SmartDashboard.putNumber("target rotation", targetRotation);



  }
}
