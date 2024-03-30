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
import frc.robot.commands.AutoPresetShooter;
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
    boolean isKillAuto = false;
    boolean isFourthNote = SmartDashboard.getBoolean("Fourth note", true);

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
          new AutoPresetShooter(shooterSubsystem, 0, Shooter.TILT_HIGH, Shooter.SHOT_SHORT),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 0,0,0,320),
          new AutoDriveByShootingCommand(Shooter.SHOT_SHORT,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, true),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 130,0.3,1,350),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 250,0.5,1,320),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
                  isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,130,1, 4,340),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,100,1, 2,340),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(3.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 290,1, 6, 320),
          new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 310,1, 2, 320),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false)):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
          );
        }
        else if(angle>15){
          System.out.println("R1");
          //1
          auto = new SequentialCommandGroup(
            new AutoPresetShooter(shooterSubsystem, 0, Shooter.TILT_HIGH, Shooter.SHOT_SHORT),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 40),
            new AutoDriveByShootingCommand(Shooter.SHOT_SHORT,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, true),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 90,0.3,0.5,20),
            new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
            new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 270,0.3,1.2,40),
            new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
          isMidfield?
            new SequentialCommandGroup(
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem,80,1, 4.5,10),
            new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 265,1, 6, 0),
            new AutoPresetShooter(shooterSubsystem, 0, 0, Shooter.SHOT_MEDIUM),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 270,1, 1, 40),
            new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
          isFourthNote?
              new SequentialCommandGroup(
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem,80,1, 5.5,-15),
              new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(3.0)
              ):
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
            ):
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
          );
        }
        else{
          System.out.println("R2");
          //2
          auto = new SequentialCommandGroup(
            new AutoPresetShooter(shooterSubsystem, 0, Shooter.TILT_HIGH, Shooter.SHOT_SHORT),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0),
            new AutoDriveByShootingCommand(Shooter.SHOT_SHORT, Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, true),
            new AutoPresetShooter(shooterSubsystem, Shooter.TILT_LOW, 0, Shooter.SHOT_MEDIUM),
            new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
            new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, Shooter.TILT_LOW, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
          isMidfield ?
            new SequentialCommandGroup(
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 120, 1, 2.5, 0),
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 90, 1, 1, 0),
              new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout( 2.0),
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 270, 1, 3, 0),
              new AutoPresetShooter(shooterSubsystem, 0, 0, Shooter.SHOT_MEDIUM),
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 300, 1, 2.7, 0),
              new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
            isFourthNote?
              new SequentialCommandGroup(
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 90, 1, 1, 0),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 130, 1, 4, -20),
                  new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout( 2.0),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 310, 1, 4, 0),
                  new AutoPresetShooter(shooterSubsystem, Shooter.TILT_LOW, 0, Shooter.SHOT_MEDIUM),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 300, 1, 2, 0),
                  new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, Shooter.TILT_LOW, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
                new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
                ) :
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 0)
            ):
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
            new AutoPresetShooter(shooterSubsystem, 0, Shooter.TILT_HIGH, Shooter.SHOT_SHORT),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 140),
            new AutoDriveByShootingCommand(Shooter.SHOT_SHORT,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, true),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 270,0.3,0.5,160),
            new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
            new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 90,0.3,1.2,140),
            new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
          isMidfield?
            new SequentialCommandGroup(
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem,280,1, 4.5,170),
            new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(2.0),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 95,1, 6, 180),
            new AutoPresetShooter(shooterSubsystem, 0, 0, Shooter.SHOT_MEDIUM),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 90,1, 1, 140),
            new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
          isFourthNote?
              new SequentialCommandGroup(
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem,280,1, 5.5,195),
              new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(3.0)
              ):
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
            ):
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
          );
        }
        else if(angle<-90 && angle>-165){
          System.out.println("B3");
          //3
          auto = new SequentialCommandGroup(
          new AutoPresetShooter(shooterSubsystem, 0, Shooter.TILT_HIGH, Shooter.SHOT_SHORT),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 0,0,0,220),
          new AutoDriveByShootingCommand(Shooter.SHOT_SHORT,Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, true),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 230,0.3,1,190),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
          new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 110,0.5,1,220),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
                  isMidfield?
          new SequentialCommandGroup(
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,230,1, 4,200),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem,260,1, 2,200),
          new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout(3.0),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 70,1, 6, 220),
          new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
          new AutoDriveCommand(swerveSubsystem,sensorSubsystem, 50,1, 2, 220),
          new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM,Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false)):
          new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
          );
        }
        else{            
          System.out.println("B2");
          //2
          auto = new SequentialCommandGroup(
            new AutoPresetShooter(shooterSubsystem, 0, Shooter.TILT_HIGH, Shooter.SHOT_SHORT),
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180),
            new AutoDriveByShootingCommand(Shooter.SHOT_SHORT, Shooter.TILT_HIGH, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, true),
            new AutoPresetShooter(shooterSubsystem, Shooter.TILT_LOW, 0, Shooter.SHOT_MEDIUM),
            new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
            new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, Shooter.TILT_LOW, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
          isMidfield ?
            new SequentialCommandGroup(
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 240, 1, 2.5, 180),
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 270, 1, 1, 180),
              new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout( 2.0),
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 90, 1, 3, 180),
              new AutoPresetShooter(shooterSubsystem, 0, 0, Shooter.SHOT_MEDIUM),
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 60, 1, 2.7, 180),
              new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
            isFourthNote?
              new SequentialCommandGroup(
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 270, 1, 1, 180),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 230, 1, 4, 200),
                  new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem).withTimeout( 2.0),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 50, 1, 3, 180),
                  new AutoPresetShooter(shooterSubsystem, Shooter.TILT_MEDIUM, 0, Shooter.SHOT_MEDIUM),
                  new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 60, 1, 2.7, 180),
                  new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, Shooter.TILT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem, false),
                new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
                ) :
              new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
            ):
            new AutoDriveCommand(swerveSubsystem, sensorSubsystem, 0, 0, 0, 180)
          );
        }
      }
    }

    return auto;
  }
}
