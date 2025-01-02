package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANcoder canCoder;

    private final Rotation2d offset;

    private final SparkPIDController steerController;
    private final SparkPIDController driveController;

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
    public SwerveModule(int moduleID, boolean driveMotorInverted, boolean steerMotorInverted, double steerOffsetRadians) {
        moduleID *= 10;
        int driveMotorId = moduleID+1;
        int steerMotorId = moduleID+2;
        int canCoderId = moduleID+3;
        
        driveMotor = new CANSparkMax(driveMotorId,MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorId,MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveMotorInverted);
        steerMotor.setInverted(steerMotorInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setSmartCurrentLimit(DriveConstants.driveCurrentLimitAmps);
        driveMotor.setSmartCurrentLimit(DriveConstants.steerCurrentLimitAmps);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        canCoder = new CANcoder(canCoderId);

        offset = new Rotation2d(steerOffsetRadians);

        steerController = steerMotor.getPIDController();
        driveController = driveMotor.getPIDController();

        steerController.setP(DriveConstants.steerkP);
        steerController.setI(DriveConstants.steerkI);
        steerController.setD(DriveConstants.steerkD);

        //set the output of the drive encoder to be in radians for linear measurement
        driveEncoder.setPositionConversionFactor(DriveConstants.driveMetersPerEncRev);

        //set the output of the drive encoder to be in radians per second for velocity measurement
        driveEncoder.setVelocityConversionFactor(DriveConstants.driveMetersPerSecPerRPM);

        //set the output of the steeration encoder to be in radians
        steerEncoder.setPositionConversionFactor(DriveConstants.steerRadiansPerEncRev);


        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
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
        double unsignedAngle = (Units.rotationsToRadians((canCoder.getAbsolutePosition().refresh().getValue())) - offset.getRadians()) % (2 * Math.PI);
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
        desiredState = SwerveModuleState.optimize(desiredState, getSteerEncAngle());


        steerController.setReference(
            calculateAdjustedAngle(
                desiredState.angle.getRadians(),//Benjy Set to -
                getSteerEncAngle().getRadians()),
            ControlType.kPosition
        );
        
        if(isOpenLoop) {
            driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kFreeMetersPerSecond);
        }
        else {
            double speedMetersPerSecond = desiredState.speedMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec;

            driveController.setReference(
                speedMetersPerSecond,
                ControlType.kVelocity,
                0, 
                DriveConstants.driveFF.calculate(speedMetersPerSecond)
            );
        }
    }

    public void setDriveCurrentLimit(int amps) {
        driveMotor.setSmartCurrentLimit(amps);
    }

    public void setSteerCurrentLimit(int amps) {
        steerMotor.setSmartCurrentLimit(amps);
    }
}