// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;


public class AutoIntakeCommand extends Command {
  /** Creates a new AutoIntakeCommand. */
  private final SwerveSubsystem swerveSubsystem;
  private final SensorSubsystem sensorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  public AutoIntakeCommand(    
    SwerveSubsystem swerveSubsystem, 
    SensorSubsystem sensorSubsystem,
    IntakeSubsystem intakeSubsystem) {
      this.sensorSubsystem = sensorSubsystem;
      this.swerveSubsystem = swerveSubsystem;
      this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Set speeds based on note position on screen
    double ySpeed = 0;
    double xSpeed = 0;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, 0);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
    intakeSubsystem.setIntakeSpeed(Constants.Intake.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.hasNote();
  }
}
