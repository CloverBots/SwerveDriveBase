package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.constants.SwerveDriveConstants.SwerveModuleConfigurations;

/**
 * Represents a Swerve Module on the robot. Contains movement and turning functionality for the wheel.
*/
public class SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;
    private final CANCoder turningEncoder;

    /** Contains the motor/encoder IDs, offsets, and other settings for the module. */
    protected SwerveModuleConfigurations config;

    private PIDController turningPidController;
    public PIDController velocityPidController;

    /**
     * Constructs a new SwerveModule. 
     * @param config The configuration for this module. This is how the SwerveModule gets its constants (Motor IDs, offsets, etc.)
     */
    public SwerveModule(SwerveModuleConfigurations config) {
        this.config = config;
        this.driveMotor = new TalonFX(config.driveMotorID);
        this.turningMotor = new CANSparkMax(config.turnMotorID, MotorType.kBrushless);
        this.turningEncoder = new CANCoder(config.CANCoderID);
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        configureCANCoder(turningEncoder);
        
        // This makes getPosition() on the turning motor encoder give it's rotation in radians.
        turningMotor.getEncoder().setPositionConversionFactor(SwerveDriveConstants.TURNING_ENCODER_TO_RAD);
        // This makes getVelocity() on the turning motor encoder give its velocity in radians/second.
        turningMotor.getEncoder().setVelocityConversionFactor(SwerveDriveConstants.TURNING_ENCODER_TO_RADS_PER_SECOND);

        driveMotor.setInverted(config.driveInverted);
        turningMotor.setInverted(true);
        turningPidController = new PIDController(
            SwerveDriveConstants.kPTurning,
            SwerveDriveConstants.kITurning,
            SwerveDriveConstants.kDTurning
        );
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
    }
    protected void setBrakeMode(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }
    /**
     * Gives the current angle of the wheel on the swerve module.
     * @return The angle, in radians from 0 to 2Pi, of the wheel.
     */
    public double getTurningPosition() {
        double pos = turningMotor.getEncoder().getPosition();
        boolean sign = pos < 0;
        //SmartDashboard.putNumber("posBefore "+config.name(), Units.radiansToDegrees(pos));
        pos = Math.abs(pos);
        pos = pos % (2*Math.PI);
        if (sign) pos = (2*Math.PI) - pos;
        return pos;
    }
    /**
     * Gives the value from the absolute encoder (CANCoder). Will always keep its angle even if the robot is turned off.
     * @return The angle from the CANCoder.
     */
    public double getAbsolutePosition() {
        return turningEncoder.getAbsolutePosition();
    }
    /**
     * Gives velocity that the wheel is turning at currently.
     * @return The turning velocity, in radians/second.
     */
    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }
    /** Resets the drive encoder to 0, and synchronizes the turning encoder. */
    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.getEncoder().setPosition(Units.degreesToRadians(getAbsolutePosition()));
    }
    /** Synchronizes the turning encoder with the CANCoder. */
    public void resetTurnEncoder() {
        turningMotor.getEncoder().setPosition(Units.degreesToRadians(getAbsolutePosition()));
    }
    /** Resets the drive encoder to 0. */
    public void resetDriveEncoder() {
        driveMotor.setSelectedSensorPosition(0);
    }
    /** Gives the current velocity of the wheel, in Meters/Second. */
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * SwerveDriveConstants.DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }
    /** Gives the current {@code SwerveModuleState} of the module. Contains the current angle and velocity of the wheel. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity(), new Rotation2d(getTurningPosition()));
    }
    /** Gives the current {@code SwerveModulePosition} of the module. Contains the total distance travelled and the current angle of the wheel.*/
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition() * SwerveDriveConstants.DRIVE_ENCODER_TO_METERS, new Rotation2d(getTurningPosition()));
    }
    /** Gives the total distance travelled by the wheel, in meters. */
    public double getDriveMotorPosition() {
        return driveMotor.getSelectedSensorPosition() * SwerveDriveConstants.DRIVE_ENCODER_TO_METERS;
    }
    /** Sets the desired state of the swerve module.
     * @param state The desired {@code SwerveModuleState}. Contains the desired velocity and angle of the wheel.
     */
    public void setDesiredState(SwerveModuleState state) {
        
        // Prevent the wheel from moving at very low speeds.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        // This will minimize the change in angle by allowing the wheel to spin backwards if needed.
        // For example, if you want to make the robot move in the opposite direction, this will make it so the wheel will simply spin backwards instead of turning the whole wheel around.
        state = SwerveModuleState.optimize(state, getState().angle);

        // Calculates the (estimated) power needed for the wheel to reach the speed needed.
        double output = (state.speedMetersPerSecond / SwerveDriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        driveMotor.set(TalonFXControlMode.PercentOutput, output);
        
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    /** Sets the speed of both motors to 0. */
    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }
    /** Configures the CANCoder. */
    private void configureCANCoder(CANCoder cancoder) {
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfig.magnetOffsetDegrees = config.encoderOffset;
        encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder.configAllSettings(encoderConfig);
    }

}