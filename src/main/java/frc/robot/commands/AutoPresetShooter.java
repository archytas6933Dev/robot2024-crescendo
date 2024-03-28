// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPresetShooter extends Command {

  private ShooterSubsystem shooterSubsystem;
  private double targetangle;
  private double initangle;
  private double shotspeed;
  /** Creates a new AutoSetShooterAngle. */
  public AutoPresetShooter(ShooterSubsystem shooterSubsystem, double targetangle, double initangle, double shotspeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetangle = targetangle;
    this.initangle = initangle;
    this.shotspeed = shotspeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if (initangle != 0) shooterSubsystem.initializeTiltPosition(initangle);
    if (targetangle != 0) shooterSubsystem.setTiltPosition(targetangle);
    if (shotspeed != 0) shooterSubsystem.setshotspeed(shotspeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
