// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
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

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() 
  {
    sensorSubsystem = new SensorSubsystem();
    swerveSubsystem = new SwerveSubsystem(sensorSubsystem);
    shooterSubsystem = Constants.Shooter.EXISTS ? new ShooterSubsystem():null;
    intakeSubsystem = Constants.Intake.EXISTS ? new IntakeSubsystem():null;

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        driverJoystick, 
        swerveSubsystem, 
        sensorSubsystem,
        shooterSubsystem,
        intakeSubsystem));

    configureBindings();

    NetworkTableInstance.getDefault().getTable("limelight-archy").getEntry("pipeline").setNumber(3);
  }

  private void configureBindings() 
  {
    new JoystickButton(driverJoystick, OIConstants.STARTBUTTON).onTrue( new InstantCommand( () -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
