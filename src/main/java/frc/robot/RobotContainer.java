// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Control;
import frc.robot.Constants.Shooter;
import frc.robot.commands.AutoDriveByShootingCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer 
{

  private final SensorSubsystem sensorSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem;

  private final Joystick driverJoystick = new Joystick(Control.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(Control.kOperatorControllerPort);


  public RobotContainer() 
  {
    sensorSubsystem = new SensorSubsystem();
    swerveSubsystem = new SwerveSubsystem(sensorSubsystem);
    shooterSubsystem = Constants.Shooter.EXISTS ? new ShooterSubsystem():null;
    intakeSubsystem = Constants.Intake.EXISTS ? new IntakeSubsystem():null;
    climberSubsystem = Constants.Climber.EXISTS ? new ClimberSubsystem():null;

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        driverJoystick,
        operatorJoystick, 
        swerveSubsystem, 
        sensorSubsystem,
        shooterSubsystem,
        intakeSubsystem,
        climberSubsystem));

    configureBindings();

    NetworkTableInstance.getDefault().getTable("limelight-archy").getEntry("pipeline").setNumber(3);
  }

  private void configureBindings() 
  {
    new JoystickButton(driverJoystick, Control.STARTBUTTON).onTrue( new InstantCommand( () -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    System.out.println("start");
    boolean isRed = sensorSubsystem.isRed;
    boolean isMidfield = SmartDashboard.getBoolean("midfield auto", true);
    boolean isKillAuto = SmartDashboard.getBoolean("kill auto", false);

    double angle = swerveSubsystem.getPos().getRotation().getDegrees();
    System.out.println(angle);
    SequentialCommandGroup auto = null;
    if(!isKillAuto){
      if(isRed){
        //red
        if(angle<345 && angle>180){
          System.out.println("R3");
          //3
          auto = new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 0,0,0,340),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 130,0.3,0.5,340),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,290,0.8, 2,355),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 70,0.8, 3, 340),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem))
          :
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
          );
        }
        else if(angle>15){
          System.out.println("R1");
          //1
          auto = new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 0,0,0,40), // deg 160
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 180,0.3,0.2,40),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
                  isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,80,1, 4.5,10),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 270,1, 6, 0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 270,1, 1, 40),

          // new AutoDriveCommand(swerveSubsystem, 250, 0.5, 2, 20),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
          );
        }
        else{
          System.out.println("R2");
          //2
          auto = new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,0,0,0,0),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          // new AutoDriveCommand(swerveSubsystem, 180, 0.4, 1.5, 135),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,110,0.8,2.5,0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,90,0.8,1.5,0),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,270,0.8,3.5,0),

          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,290,0.8,2,0),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
          );

        }
      }
      else{
        //blue
        if(angle>90 && angle<170){
          System.out.println("B1");
          //1
          auto = new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 0,0,0,140), // deg 160
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 180,0.3,0.2,140),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
                  isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,280,1, 4.5,180),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 90,1, 6, 180),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 90,1, 1, 140),

          // new AutoDriveCommand(swerveSubsystem, 250, 0.5, 2, 20),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
          );
        }
        else if(angle<-90 && angle>-165){
          System.out.println("B3");
          //3
          auto = new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 0,0,0,160),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 230,0.3,0.5,160),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
                  isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,110,0.8, 2,175),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 250,0.8, 3, 160),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
          );
        }
        else{            
          System.out.println("B2");
          //2
          auto = new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,0,0,0,180),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          // new AutoDriveCommand(swerveSubsystem, 180, 0.4, 1.5, 135),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
          isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,250,0.8,3,180),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,270,0.8,1.5,180),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,90,0.8,3.5,180),

          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,70,0.8,2,180),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)
          ):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
          );
        }
      }
    }
    //red middle
    // SequentialCommandGroup auto = new SequentialCommandGroup(
    //   new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
    //   new AutoDriveCommand(swerveSubsystem,-90,0.5,0.2,0),

    //   new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
    //   new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
    //   new AutoDriveCommand(swerveSubsystem, 45, 0.3, 2, 225),
    //   new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
    //   new AutoDriveCommand(swerveSubsystem, -90, 0.3, 0.5, -45),
    //   new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)
    //   // new AutoDriveCommand(swerveSubsystem, -90, 0.3, 0.5, 45),

    //   );
    // auto.addCommands(new AutoDriveCommand(swerveSubsystem,0,0.2,1,0));

    //blue 1
    // SequentialCommandGroup auto = new SequentialCommandGroup(
    //   new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
    //   new AutoDriveCommand(swerveSubsystem, 180, 0.4, 1.5, 135),
    //   new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
    //   new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
    // );



    return auto;
  }
}
