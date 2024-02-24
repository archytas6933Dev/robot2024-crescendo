// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer 
{

  private final SensorSubsystem sensorSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final Joystick driverJoystick = new Joystick(Control.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(Control.kOperatorControllerPort);


  public RobotContainer() 
  {
    sensorSubsystem = new SensorSubsystem();
    swerveSubsystem = new SwerveSubsystem(sensorSubsystem);
    shooterSubsystem = Constants.Shooter.EXISTS ? new ShooterSubsystem():null;
    intakeSubsystem = Constants.Intake.EXISTS ? new IntakeSubsystem():null;

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        driverJoystick,
        operatorJoystick, 
        swerveSubsystem, 
        sensorSubsystem,
        shooterSubsystem,
        intakeSubsystem));

    configureBindings();

    NetworkTableInstance.getDefault().getTable("limelight-archy").getEntry("pipeline").setNumber(3);
  }

  private void configureBindings() 
  {
    new JoystickButton(driverJoystick, Control.STARTBUTTON).onTrue( new InstantCommand( () -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    SequentialCommandGroup auto = new SequentialCommandGroup(
      new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
      new AutoDriveCommand(swerveSubsystem,-90,0.5,0.2,0),

      new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
      new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem),
      new AutoDriveCommand(swerveSubsystem, 45, 0.3, 2, 225),
      new AutoIntakeCommand(swerveSubsystem, sensorSubsystem, intakeSubsystem),
      new AutoDriveCommand(swerveSubsystem, -90, 0.3, 0.5, -45),
      new AutoDriveByShootingCommand(Shooter.SHOT_MEDIUM, swerveSubsystem, sensorSubsystem, intakeSubsystem, shooterSubsystem)
      // new AutoDriveCommand(swerveSubsystem, -90, 0.3, 0.5, 45),

      );
    // auto.addCommands(new AutoDriveCommand(swerveSubsystem,0,0.2,1,0));


    return auto;
  }
}
