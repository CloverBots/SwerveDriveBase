// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drives the robot to a given X / Y position, and angle. This is relative to the robot's initial position.
 */
public class DriveToDistanceCommand extends CommandBase {
  private SwerveSubsystem swerveSubsystem;

  private PIDController driveDistanceControllerX = new PIDController(7.0, 0.25, 0.1);
  private PIDController driveDistanceControllerY = new PIDController(7.0, 0.25, 0.1);
  private PIDController rotationController = new PIDController(Math.PI, 0, 0);

  private Timer timer;
  private double timeout;
  /**
   * Drives the robot to a given X / Y position, and angle. This is relative to the robot's initial position.
   * @param swerveSubsystem
   * @param xPos The X position you want the robot to go to. This is forwards/backwards.
   * @param yPos The Y position you want the robot to go to. This is left/right.
   * @param angle The angle, in Radians, that the robot should be at.
   * @param timeout The maximum amount of time, in seconds, that this command should run for. The command will end if the timeout is reached and if it hasn't ended already.
   */
  public DriveToDistanceCommand(SwerveSubsystem swerveSubsystem, double xPos, double yPos, double angle, double timeout) {
    addRequirements(swerveSubsystem);
    this.timer = new Timer();
    this.timeout = timeout;
    this.swerveSubsystem = swerveSubsystem;
    driveDistanceControllerX.setSetpoint(xPos);
    driveDistanceControllerX.setTolerance(0.005);
    driveDistanceControllerY.setSetpoint(yPos);
    driveDistanceControllerY.setTolerance(0.005);
    rotationController.setSetpoint(angle);
    rotationController.setTolerance(0.005);
    rotationController.enableContinuousInput(0, 2*Math.PI); // Sets the PID to treat zero and 2 pi as the same value.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Status", true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = driveDistanceControllerX.calculate(swerveSubsystem.getPose().getX());
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), SwerveDriveConstants.AUTO_MAX_SPEED), xSpeed);

    double ySpeed = driveDistanceControllerY.calculate(swerveSubsystem.getPose().getY());
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), SwerveDriveConstants.AUTO_MAX_SPEED), ySpeed);

    double dTheta = rotationController.calculate(Units.degreesToRadians(swerveSubsystem.getHeading()));
    dTheta = Math.copySign(Math.min(Math.abs(dTheta),2), dTheta);
    swerveSubsystem.setSpeed(xSpeed, ySpeed, -dTheta, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    SmartDashboard.putBoolean("Status", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveDistanceControllerX.atSetpoint() && driveDistanceControllerY.atSetpoint()) 
    || (timer.get() >= timeout);
  
  }
}
