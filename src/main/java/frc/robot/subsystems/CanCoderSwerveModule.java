package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

@Logged
public class CanCoderSwerveModule extends SubsystemBase {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    @NotLogged
    private final SparkClosedLoopController driveController;
    @NotLogged
    private final SparkClosedLoopController steerController;

    @NotLogged
    private final SparkMaxConfig driveConfig = new SparkMaxConfig();
    @NotLogged
    private final SparkMaxConfig steerConfig = new SparkMaxConfig();

    private final CANcoder canCoder;
    @NotLogged
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    private final Rotation2d offset;

    /**
     * Create a new SwerveModule.
     * 
     * <p> Tracks and controls a single swerve module
     * 
     * <p> IDs: Front left - 1, front right - 2, back left - 3, back right - 4
     * 
     * 
     * @param moduleId           Module number, used to determine SPARKMax and CanCoder IDs
     * @param driveMotorInverted Drive NEO is inverted.
     * @param steerMotorInverted Steer NEO is inverted.
     * @param steerOffsetRadians Offset of CANCoder reading from forward.
     */
    public CanCoderSwerveModule(int moduleID, boolean driveMotorInverted, boolean steerMotorInverted, double steerOffsetRadians) {
        moduleID *= 10;
        int driveMotorId = moduleID+1;
        int steerMotorId = moduleID+2;
        int canCoderId = moduleID+3;
        
        driveMotor = new SparkMax(driveMotorId,MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId,MotorType.kBrushless);

        driveConfig
            .inverted(driveMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.driveCurrentLimitAmps);
        driveConfig.encoder
            //set the output of the drive encoder to be in radians for linear measurement
            .positionConversionFactor(DriveConstants.driveMetersPerEncRev)
            //set the output of the drive encoder to be in radians per second for velocity measurement
            .velocityConversionFactor(DriveConstants.driveMetersPerSecPerRPM);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        steerConfig
            .inverted(steerMotorInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(DriveConstants.steerCurrentLimitAmps);
        steerConfig.encoder
            //set the output of the steeration encoder to be in radians
            .positionConversionFactor(DriveConstants.steerRadiansPerEncRev);
        steerConfig.closedLoop
            .pid(DriveConstants.steerkP,
                 DriveConstants.steerkI,
                 DriveConstants.steerkD);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        steerController = steerMotor.getClosedLoopController();
        driveController = driveMotor.getClosedLoopController();

        canCoder = new CANcoder(canCoderId);

        offset = new Rotation2d(steerOffsetRadians);

        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(canCoderConfig);

        initSteerOffset();
    }
    
    /**
    * Initializes the steer motor encoder to the value of the CANCoder, accounting for the offset.
    */
    public void initSteerOffset() {
       steerEncoder.setPosition(getCanCoderAngle().getRadians());
    }
    
    /**
     * Returns the current angle of the module between 0 and 2 * PI.
     * 
     * @return The current angle of the module between 0 and 2 * PI.
     */
    public Rotation2d getCanCoderAngle() {        
        double unsignedAngle =
            canCoder.getAbsolutePosition().getValue()
            .minus(offset.getMeasure())
            .in(Radians) % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);
    }

        /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), getCanCoderAngle());
    }

    /**
     * Resets the distance traveled by the module.
     */
    public void resetDistance() {
        driveEncoder.setPosition(0.0);
    }

    /**
     * Returns the current drive distance of the module.
     * 
     * @return The current drive distance of the module.
     */
    public double getDriveDistanceMeters() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the current absolute angle of the module from the steeration motor encoder.
     * 
     * @return The current absolute angle of the module.
     */
    public Rotation2d getSteerEncAngle() {
        return new Rotation2d(steerEncoder.getPosition());
    }

    /**
     * Returns the current velocity of the module from the drive motor encoder.
     * 
     * @return The current velocity of the module in meters per second.
     */
    public double getCurrentVelocityMetersPerSecond() {
        return driveEncoder.getVelocity();
    }

    public double getRelativeVelocityMetersPerSecond(double thetaRad) {
        double rel = getCanCoderAngle().getDegrees() % 90.0;
        if(rel > 90.0 && rel < 270.0) rel *= -1.0;
        return getCurrentVelocityMetersPerSecond() * (rel / 90.0);
    }

    /**
     * Calculates the angle motor setpoint based on the desired angle and the current angle measurement.
     */
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {
        double modAngle = currentAngle % (2.0 * Math.PI);
        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        double newTarget = targetAngle + currentAngle - modAngle;
        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;
        return newTarget;
    }


    /**
     * Sets the desired state of the swerve module and optimizes it.
     * <p>If closed-loop, uses PID and a feedforward to control the speed.
     * If open-loop, sets the speed to a percentage. Open-loop control should
     * only be used if running an autonomour trajectory.
     *
     * @param desiredState Object that holds a desired linear and steerational setpoint.
     * @param isOpenLoop True if the velocity control is open- or closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        
        // Optimizes speed and angle to minimize change in heading
        // (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
        desiredState.optimize(getSteerEncAngle());


        steerController.setReference(
            calculateAdjustedAngle(
                desiredState.angle.getRadians(),//Benjy Set to -
                getSteerEncAngle().getRadians()),
            ControlType.kPosition
        );
        
        if(isOpenLoop) {
            driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.freeMetersPerSecond);
        }
        else {
            double speedMetersPerSecond = desiredState.speedMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec;

            driveController.setReference(
                speedMetersPerSecond,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                DriveConstants.driveFF.calculate(speedMetersPerSecond)
            );
        }
    }

    public void setDriveCurrentLimit(int amps) {
        driveConfig.smartCurrentLimit(amps);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSteerCurrentLimit(int amps) {
        driveConfig.smartCurrentLimit(amps);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void flashMotorConfigs() {
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void isTestMode(boolean isTestMode) {
    }

}