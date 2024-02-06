package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import javax.print.CancelablePrintJob;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveDrive extends SubsystemBase {
    // Initialize new swerve module objects
    private final SwerveModule frontLeftMod = 
        new SwerveModule(
            1,
                CANDevices.frontLeftDriveInverted,
                CANDevices.frontLeftSteerInverted,
            DriveConstants.frontLeftModuleOffset
        );

    private final SwerveModule frontRightMod = 
        new SwerveModule(
            2,
                CANDevices.frontRightDriveInverted,
                CANDevices.frontRightSteerInverted,
            DriveConstants.frontRightModuleOffset
        );

    private final SwerveModule backLeftMod = 
        new SwerveModule(
            3,
                CANDevices.backLeftDriveInverted,
                CANDevices.backLeftSteerInverted,
            DriveConstants.backLeftModuleOffset
        );

    private final SwerveModule backRightMod = 
        new SwerveModule(
            4,
                CANDevices.frontRightDriveInverted,
                CANDevices.frontRightSteerInverted,
            DriveConstants.frontRightModuleOffset
        );
    
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private boolean isLocked = false;
    private boolean isFieldOriented = false;
    private double speedFactor = 0.3; //Speed
    private boolean speedChanged = false;
    private boolean isTracking = false;
    // Odometry for the robot, measured in meters for linear motion and radians for rotational motion
    // Takes in kinematics and robot angle for parameters
    private SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d()
        );

    public boolean isLocked() {
        return isLocked;
    }
    
    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    public double getSpeedFactor() {
        return speedFactor;
    }
    public void setSpeedFactor(double newSpeedFactor) {
        speedChanged = true;
        this.speedFactor = newSpeedFactor;
        SmartDashboard.updateValues();
    }
    /**
     * Increase driving speed by percentage
     * @param difference Amount of speed to increase by
     */
    public Command speedUpCommand(double difference) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                setSpeedFactor(speedFactor+difference);
            });
    }
    /**
     * Increase driving speed by percentage
     * @param difference Amount of speed to increase by
     */
    public Command slowDownCommand(double difference) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                setSpeedFactor(speedFactor-difference);
            });
    }

    public boolean isTracking() {
        return isTracking;
    }
    public void setIsTracking(boolean isTracking) {
        this.isTracking = isTracking;
    }

    /**
     * Stops the driving of the robot.
     * <p>Sets the drive power of each module to zero while maintaining module headings.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0, isFieldOriented);
    }

    public void lock() {
        isLocked = true;
    }

    /**
     * Lock wheels
     *
     * @return a command
     */
    public Command lockCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                lock();
            });
    }
    
    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control (closed-loop) to control the linear and rotational values for the modules.
     * 
     * @param moduleStates the module states to set.
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], false);
        frontRightMod.setDesiredState(moduleStates[1], false);
        backLeftMod.setDesiredState(moduleStates[2], false);
        backRightMod.setDesiredState(moduleStates[3], false);
    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control to control the linear and rotational values for the modules.
     * 
     * @param moduleStates the module states to set.
     */
    public void setModuleStatesAuto(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        backLeftMod.setDesiredState(moduleStates[2], true);
        backRightMod.setDesiredState(moduleStates[3], true);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStatesAuto(DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds));
    }

        /**
     * Returns an array of module states.
     * 
     * @return An array of SwerveModuleState.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftMod.getCurrentVelocityMetersPerSecond(), frontLeftMod.getSteerEncAngle()),
            new SwerveModuleState(frontRightMod.getCurrentVelocityMetersPerSecond(), frontRightMod.getSteerEncAngle()),
            new SwerveModuleState(backLeftMod.getCurrentVelocityMetersPerSecond(), backLeftMod.getSteerEncAngle()),
            new SwerveModuleState(backRightMod.getCurrentVelocityMetersPerSecond(), backRightMod.getSteerEncAngle())
        };
    }

    /**
     * Returns an array of module positions.
     * 
     * @return An array of SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftMod.getPosition(),
            frontRightMod.getPosition(),
            backLeftMod.getPosition(),
            backRightMod.getPosition()
        };
    }

    /**
     * Returns the current heading of the robot from the gyro.
     * 
     * @return The current heading of the robot.
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-navX.getYaw());
    }
    
    /**
     * Returns the current pitch of the robot from the gyro.
     * 
     * @return The current pitch of the robot.
     */
    public double getPitchDegrees() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return navX.getPitch();
    }

    /**
     * Returns the current roll of the robot from the gyro.
     * 
     * @return The current roll of the robot.
     */
    public double getRollDegrees() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return navX.getRoll();
    }

    /**
     * Sets the gyro heading to zero.
     */
    public void resetHeading() {
        navX.zeroYaw();
    }

    public void setDriveCurrentLimit(int amps) {
        frontLeftMod.setDriveCurrentLimit(amps);
        frontRightMod.setDriveCurrentLimit(amps);
        backLeftMod.setDriveCurrentLimit(amps);
        backRightMod.setDriveCurrentLimit(amps);
    }

    public Command driveCommand(
        DoubleSupplier driveSupplier, 
        DoubleSupplier strafeSupplier, 
        DoubleSupplier rotSupplier,
        boolean isFieldRelative) {
            return runOnce(
                () -> {
                    double drive = driveSupplier.getAsDouble();
                    drive *= Math.abs(drive);
            
                    double strafe = strafeSupplier.getAsDouble();
                    strafe *= Math.abs(strafe);
            
                    double rot = rotSupplier.getAsDouble();
                    rot *= Math.abs(rot);

                    this.drive(
                        -drive,
                        -strafe,
                        -rot,
                        isFieldRelative
                    );
                });
    
        

    }

    /**
     * Inputs drive values into the swerve drive base.
     * 
     * @param driveX The desired forward/backward lateral motion, in meters per second.
     * @param driveY The desired left/right lateral motion, in meters per second.
     * @param rotation The desired rotational motion, in radians per second.
     * @param isFieldOriented whether driving is field- or robot-oriented.
     */
    public void drive(double driveX, double driveY, double rotation, boolean isFieldRelative) {  
        if(driveX != 0.0 || driveY != 0.0 || rotation != 0.0) isLocked = false;
        isFieldOriented = isFieldRelative;
        
        if(isLocked) {
            setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            });
        }
        else {
            // Reduces the speed of the drive base for "turtle" or "sprint" modes.
            driveX *= speedFactor;
            driveY *= speedFactor;
            rotation *= speedFactor; //TODO: make individually modifiable

            // Represents the overall state of the drive base.
            ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveX, driveY, rotation, getHeading())
                : new ChassisSpeeds(driveX, driveY, rotation); //If you put - on rotation it the wheels goes backwards not the angle

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states.
            SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds); 
            
            // Makes sure the wheels don't try to spin faster than the maximum speed possible
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

            setModuleStates(states);
        }
    }

    /**
     * Constructs a new SwerveSys.
     * 
     * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
     */
    public SwerveDrive() {
        // Resets the measured distance driven for each module
        // resetDriveDistances();

        // resetPose();
        // SmartDashboard.putNumber("Speed Percent", speedFactor);

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Updates the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("front left CANcoder", frontLeftMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("front right CANcoder", frontRightMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear left CANcoder", backLeftMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear right CANcoder", backRightMod.getCanCoderAngle().getDegrees());

        if (speedChanged) speedChanged = false;
        else speedFactor = SmartDashboard.getNumber("Speed Percent", speedFactor);
    }
    

}
