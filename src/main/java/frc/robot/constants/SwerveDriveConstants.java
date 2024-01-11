// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Constant values relating to the Swerve Drive */
public class SwerveDriveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

    /** The driving gear ratio for the swerve module (MK4i L3). This is how many times the drive motor has to turn in order for the wheel to make 1 rotation. */
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    /** The driving gear ratio for the Swerve Module (MK4i). This is how many times the turning motor has to turn in order for the module to make 1 full rotation. */
    public static final double TURNING_GEAR_RATIO = 150.0/7.0;

    // Length of the robot chassis, front to back
    public static final double wheelBase = Units.inchesToMeters(18.5); //24
    // Width of the robot chassis, left to right
    public static final double trackWidth = Units.inchesToMeters(18.5);
    
    /** The PHYSICAL maximum speed of the robot, if all motors were running at max power. About 5.5435 m/s*/
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (1/DRIVE_GEAR_RATIO) * (6380.0/60) * WHEEL_CIRCUMFERENCE;
    
    /** The maximum speed of the robot, in meters per second during TeleOp. Use this to limit the speed when using a controller.*/
    public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND; //Max is 5.5435
    public static final double TELEOP_SLOW_SPEED_METERS_PER_SECOND = 1;
    
    /** Maximum speed for the robot's turning. */
    public static final double teleOpMaxAngularSpeed = 3 * (2 * Math.PI);
    public static final double teleOpSlowAngularSpeed = 1 * (2 * Math.PI);
    /** The maximum angular acceleration for the robot's turning. */
    public static final double teleOpMaxAngularAccelerationUnitsPerSecond = 5;
    /** The maximum acceleration for the robot's X and Y movement. */
    public static final double teleOpMaxAccelerationMetersPerSecond = 5;

    public static final double AUTO_MAX_SPEED = 3;

    /** Multiply the output of {@code getSelectedSensorPosition()} by this to get the total distance travelled, in meters, on a swerve module. */
    public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));

    /** Multiply the output of {@code getSelectedSensorVelocity()} by this to get the current velocity, in meters per second, on a swerve module. */
    public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE) / (2048.0 * 60 * DRIVE_GEAR_RATIO);

    // Used for position and velocity conversions on the NEO turning motor on the swerve modules.
    public static final double TURNING_ENCODER_TO_RAD = (2 * Math.PI) / TURNING_GEAR_RATIO;
    public static final double TURNING_ENCODER_TO_DEG = (360) / TURNING_GEAR_RATIO;
    public static final double TURNING_ENCODER_TO_RADS_PER_SECOND = TURNING_ENCODER_TO_RAD / 60;

    public static final double kPTurning = 0.3;
    public static final double kITurning = 0;
    public static final double kDTurning = 0;

    /**
     * Contains the configuration info for each swerve module.
     */
    public static enum SwerveModuleConfigurations {
        FRONT_LEFT(10, 14, 18, -62.92, true),
        FRONT_RIGHT(11, 15, 19, -241.35, false),
        BACK_RIGHT(12, 16, 20, -112.58, false),
        BACK_LEFT(13, 17, 21, -3.69, true);

        public int driveMotorID;
        public int turnMotorID;
        public int CANCoderID;
        public double encoderOffset;
        public boolean driveInverted;

        private SwerveModuleConfigurations(int driveMotorID, int turnMotorID, int CANCoderID, double encoderOffset, boolean driveInverted) {
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.CANCoderID = CANCoderID;
            this.encoderOffset = encoderOffset;
            this.driveInverted = driveInverted;
        }
    }

    /**
     * This calculates the exact speed and rotation of every swerve module needed to make the robot go in a specific direction and rotation.
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2)
    );
}
