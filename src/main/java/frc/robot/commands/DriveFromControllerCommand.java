package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drives the robot using input from the controller.
 */
public class DriveFromControllerCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> leftStickX, leftStickY, rightStickX, rightStickY, crawlTrigger, slowRotate;
    private final Supplier<Boolean> yButton, bButton, aButton, xButton;
    private final Supplier<Integer> dPad;
    private final SlewRateLimiter translationLimiter, turningLimiter;
    private boolean fieldOriented = true;
    private boolean pointedTurning = false;

    private final double JOYSTICK_DEADZONE = 0.05;
    private final double POINTED_JOYSTICK_DEADZONE = 0.5;

    private boolean fieldOrientedCache, pointedModeCache = false;
    private PIDController rotationController;

    public DriveFromControllerCommand(
            SwerveSubsystem swerveSubsystem,
            Supplier<Double> leftStickX,
            Supplier<Double> leftStickY,
            Supplier<Double> rightStickX,
            Supplier<Double> rightStickY,
            Supplier<Boolean> yButton,
            Supplier<Boolean> bButton,
            Supplier<Boolean> aButton,
            Supplier<Boolean> xButton,
            Supplier<Double> crawlTrigger,
            Supplier<Double> slowRotate,
            Supplier<Integer> dPad) {
        this.swerveSubsystem = swerveSubsystem;

        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
        this.rightStickY = rightStickY;

        this.yButton = yButton;
        this.bButton = bButton;
        this.aButton = aButton;
        this.xButton = xButton;

        this.crawlTrigger = crawlTrigger;
        this.slowRotate = slowRotate;
        this.dPad = dPad;

        this.translationLimiter = new SlewRateLimiter(SwerveDriveConstants.teleOpMaxAccelerationMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(SwerveDriveConstants.teleOpMaxAngularAccelerationUnitsPerSecond);

        this.rotationController = new PIDController(0.065, 0.03, 0.005); // 0.017, 0, 0
        this.rotationController.enableContinuousInput(0, 360);
        
        // this.pointedRotationController = new PIDController(0.07, 0.02, 0);
        // this.pointedRotationController.enableContinuousInput(0, 360);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        // Start calculating our speeds
        ChassisSpeeds speeds = calculateSpeeds();

        // Handle all the toggle buttons on the joystick
        handleToggleButtons();

        SmartDashboard.putBoolean("Field Oriented", fieldOriented);

        // Feed the calculated speeds to the swerve subsystem
        swerveSubsystem.setSpeed(speeds, fieldOriented);
    }
    /**
     * Correctly handles all the toggle buttons on the controller (field oriented, point mode, etc.)
     */
    private void handleToggleButtons() {
        if ((dPad.get() == 0) && fieldOrientedCache == false) {
            fieldOriented = !fieldOriented;
        }
        fieldOrientedCache = dPad.get() == 0;

        if ((dPad.get() == 90) && pointedModeCache == false) {
            pointedTurning = !pointedTurning;
        }
        pointedModeCache = dPad.get() == 90;
    }
    /**
     * Calculates the translation (X/Y) speeds and turning speeds from the controller, depending on the current mode.
     * @return
     */
    private ChassisSpeeds calculateSpeeds() {
        double rotationSpeed;

        // Uses the ABYX buttons if any of them are pressed
        if (isTurnButtonPressed()) rotationSpeed = calculateTurningSpeedHotkey();
        
        // If none of those buttons are pressed, check to see if pointed turning is enabled.
        else if (pointedTurning) {
            rotationSpeed = calculateTurningSpeedPointed(rightStickX.get(), rightStickY.get());
        }

        // If none of those above are true, use the normal controls.
        else rotationSpeed = calculateTurningSpeedsNormal(rightStickX.get());
        
        // Calculate the Translation speeds
        double[] xySpeeds = calculateTranslationSpeeds(leftStickX.get(), leftStickY.get());

        // Contains the calculated translation and rotation speeds. Since the X and Y directions are the opposite in WPILib compared to the controller, we swap them here.
        return new ChassisSpeeds(xySpeeds[1], xySpeeds[0], rotationSpeed);
    }
    private double calculateTurningSpeedsNormal(double turningSpeed) {

        // Apply a deadzone. This will prevent the robot from moving at very small values
        turningSpeed = Math.abs(turningSpeed) > JOYSTICK_DEADZONE ? turningSpeed : 0.0;

        double turningSpeedMultiplier; // The max speed of the robot if the stick is pressed all the way.

        // Slows both rotation and translation if the crawl trigger is pressed.
        if (crawlTrigger.get() >= 0.5) {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpSlowAngularSpeed;
        } else {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpMaxAngularSpeed;
        }
        // If the slow rotate trigger is pressed, it will only slow down the rotation.
        if (slowRotate.get() >= 0.5 || crawlTrigger.get() >= 0.5) {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpSlowAngularSpeed;
        } else {
            turningSpeedMultiplier = SwerveDriveConstants.teleOpMaxAngularSpeed;
        }
        turningSpeed = turningLimiter.calculate(turningSpeed);
        turningSpeed *= turningSpeedMultiplier;
        return turningSpeed;
    }
    /**
     * Tries to point the robot in the same direction of the right joystick.
     * @param rx The X value of the joystick.
     * @param ry The Y value of the joystick
     * @return The turning speed calculated by the PID controller
     */
    private double calculateTurningSpeedPointed(double rx, double ry) {
        ry = -ry;
        rx = -rx;
        // Apply a deadzone to the joystick
        if (Math.hypot(rx, ry) < POINTED_JOYSTICK_DEADZONE) return 0;
        double angle = Math.atan2(rx, ry);
        angle *= (180/Math.PI);
        boolean sign = angle < 0;
        angle = Math.abs(angle);
        angle = angle % 360;
        if (sign) angle = 360-angle;
        SmartDashboard.putNumber("Joystick/Angle", angle);
        return -rotationController.calculate(swerveSubsystem.getHeading(), angle);
    }
    /**
     * Automatically points the robot in the four cardinal directions (0, 90, 180, and 270 degrees) relative to the A B Y X buttons.
     * For example, pressing Y (which is on top of the four buttons) will point the robot forward, or to 0 degrees.
     * @return The calculated turning speed
     */
    private double calculateTurningSpeedHotkey() {
        int angle; // The desired angle of the robot
        if (yButton.get()) {
            angle = 0;
        }
        else if (bButton.get()) {
            angle = 270;
        }
        else if (aButton.get()) {
            angle = 180;
        }
        else if (xButton.get()) {
            angle = 90;
        } 
        
        else return 0; // Returns a speed of 0 if none of the buttons are pressed.
        SmartDashboard.putNumber("Joystick/Angle", angle);
        return -rotationController.calculate(swerveSubsystem.getHeading(), angle);
    }
    /**
     * Calculates the Translation (X and Y) speeds of the robot from the controller joystick.
     * @param xSpeed The raw X value of the joystick.
     * @param ySpeed The raw Y value of the joystick.
     * @return A double[] array of length 2, containing the calculated X and Y speeds respectively.
     */
    private double[] calculateTranslationSpeeds(double xSpeed, double ySpeed) {
        double[] xySpeeds = new double[2]; // Will contain the calculated X and Y speeds

        double speedMultiplier; // The maximum speed of the robot in any direction if the joystick is pressed all the way

        // Max speed decresed if crawl trigger is pressed
        if (crawlTrigger.get() >= 0.5) speedMultiplier = SwerveDriveConstants.TELEOP_SLOW_SPEED_METERS_PER_SECOND;

        // Use default max speed if it is not pressed
        else speedMultiplier = SwerveDriveConstants.TELEOP_MAX_SPEED_METERS_PER_SECOND;
        
        /* 
        * This is the Magnitude. It is the hypotenuse of the X and Y speeds.
        * It can be thought of as the distance the joystick is from the center when it is pressed.
        * If you press the joystick all the way forward, it will always be 1.0 no matter what direction it is.
        * This is used to make sure that speed calculations, such as scaling and the deadzones, are always consistant regardless of the direction the robot is travelling in.
        */
        double magnitude = Math.hypot(ySpeed, xSpeed);

        // Apply the deadzone. This will prevent the robot from moving at very small values
        magnitude = Math.abs(magnitude) > JOYSTICK_DEADZONE ? magnitude : 0.0;

        // Most controllers are not completely circular (meaning the magnitude can go above 1.0 in certain directions), so we cap it at 1.0.
        magnitude = Math.min(magnitude, 1);

        // Apply a power curve. This makes it so that the robot will move slower at lower inputs compared to if it weren't. This makes small movements easier. 
        magnitude = Math.pow(magnitude, 3);

        // Limits the acceleration for translation
        magnitude = translationLimiter.calculate(magnitude);

        magnitude *= speedMultiplier; // Scales the magnitude
        
        // Multiply the raw X and Y inputs by the magnitude to get the correct X and Y translation speeds
        xSpeed *= magnitude;
        ySpeed *= magnitude;
        
        xySpeeds[0] = xSpeed;
        xySpeeds[1] = ySpeed;

        return xySpeeds;
    }

    /**
     * Tells if any of the ABYX buttons is currently pressed.
     * @return {@code true} if any one of these buttons are pressed.
     */
    private boolean isTurnButtonPressed() {
        return aButton.get() || bButton.get() || yButton.get() || xButton.get();
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules(); // Stop all swerve modules if the command ends.
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}