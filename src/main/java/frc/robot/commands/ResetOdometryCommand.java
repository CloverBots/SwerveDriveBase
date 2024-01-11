// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Resets the odometer on the {@link SwerveSubsystem}.
*/
public class ResetOdometryCommand extends CommandBase {

  SwerveSubsystem swerveSubsystem;
  
  /** Creates a new ResetOdometryCommand. */
  public ResetOdometryCommand(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.resetOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
