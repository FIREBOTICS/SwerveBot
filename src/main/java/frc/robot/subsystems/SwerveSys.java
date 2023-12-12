package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class SwerveSys extends SubsystemBase {

    // Initializes swerve module objects
    private final SwerveModule frontLeftMod = 
        new SwerveModule(
            CANDevices.frontLeftDriveMotorId,
            CANDevices.frontLeftSteerMotorId,
            CANDevices.frontLeftSteerEncoderId,
            DriveConstants.frontLeftModOffset,
            CANDevices.frontLeftReversed
        );

    private final SwerveModule frontRightMod = 
        new SwerveModule(
            CANDevices.frontRightDriveMotorId,
            CANDevices.frontRightSteerMotorId,
            CANDevices.frontRightSteerEncoderId,
            DriveConstants.frontRightModOffset,
            CANDevices.frontRightReversed
        );

    private final SwerveModule rearLeftMod = 
        new SwerveModule(
            CANDevices.rearLeftDriveMotorId,
            CANDevices.rearLeftSteerMotorId,
            CANDevices.rearLeftSteerEncoderId,
            DriveConstants.rearLeftModOffset,
            CANDevices.rearLeftReversed
        );

    private final SwerveModule rearRightMod = 
        new SwerveModule(
            CANDevices.rearRightDriveMotorId,
            CANDevices.rearRightSteerMotorId,
            CANDevices.rearRightSteerEncoderId,
            DriveConstants.rearRightModOffset,
            CANDevices.rearRightReversed
        );

    private boolean isLocked = false;
    public boolean isLocked() {
        return isLocked;
    }

    private boolean isFieldOriented = false;
    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    private double speedFactor = 0.3; //Speed
    public double getSpeedFactor() {
        return speedFactor;
    }
    public void setSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
    }
    public void increaseSpeedFactor(double difference) {
        this.speedFactor += difference;
    }
    public void decreaseSpeedFactor(double difference) {
        System.out.println("Hello there3");
        this.speedFactor -= difference;
    }

    private boolean isTracking = false;
    public boolean isTracking() {
        return isTracking;
    }
    public void setIsTracking(boolean isTracking) {
        this.isTracking = isTracking;
    }

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    // Odometry for the robot, measured in meters for linear motion and radians for rotational motion
    // Takes in kinematics and robot angle for parameters

    private SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d()
        );

    /**
     * Constructs a new SwerveSys.
     * 
     * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
     */
    public SwerveSys() {
        // Resets the measured distance driven for each module
        resetDriveDistances();

        resetPose();
        SmartDashboard.putNumber("Speed Percent", speedFactor);

    }
 
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Updates the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("front left CANcoder", frontLeftMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("front right CANcoder", frontRightMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear left CANcoder", rearLeftMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear right CANcoder", rearRightMod.getCanCoderAngle().getDegrees());

        speedFactor = SmartDashboard.getNumber("Speed Percent", speedFactor);
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
     * Stops the driving of the robot.
     * <p>Sets the drive power of each module to zero while maintaining module headings.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0, isFieldOriented);
    }

    // TODO: javadoc
    public void lock() {
        isLocked = true;
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
        rearLeftMod.setDesiredState(moduleStates[2], false);
        rearRightMod.setDesiredState(moduleStates[3], false);
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
        rearLeftMod.setDesiredState(moduleStates[2], true);
        rearRightMod.setDesiredState(moduleStates[3], true);
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
            new SwerveModuleState(rearLeftMod.getCurrentVelocityMetersPerSecond(), rearLeftMod.getSteerEncAngle()),
            new SwerveModuleState(rearRightMod.getCurrentVelocityMetersPerSecond(), rearRightMod.getSteerEncAngle())
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
            rearLeftMod.getPosition(),
            rearRightMod.getPosition()
        };
    }

    /**
     * @return The current estimated position of the robot on the field
     * based on drive encoder and gyro readings.
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets the current pose.
     */
    public void resetPose() {
        resetDriveDistances();
        resetHeading();

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            new Rotation2d(),
            getModulePositions(),
            new Pose2d()
        );
    }

    public void setHeading(Rotation2d heading) {
        double desiredYaw = Math.abs(heading.getDegrees() % 360.0);
        navX.setAngleAdjustment(-desiredYaw);
        
    }

    /**
     * Sets the pose of the robot.
     * 
     * @param pose The pose to set the robot to.
     */
    public void setPose(Pose2d pose) {
        setHeading(pose.getRotation());

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            pose.getRotation(),
            getModulePositions(),
            pose
        );
    }

    /**
     * Resets the measured distance driven for each module.
     */
    public void resetDriveDistances() {
        frontLeftMod.resetDistance();
        frontRightMod.resetDistance();
        rearLeftMod.resetDistance();
        rearRightMod.resetDistance();
    }

    /**
     * Returns the average distance driven of each module to get an overall distance driven by the robot.
     * 
     * @return The overall distance driven by the robot in meters.
     */
    public double getAverageDriveDistanceMeters() {
        return (
            (frontLeftMod.getDriveDistanceMeters()
            + frontRightMod.getDriveDistanceMeters()
            + rearLeftMod.getDriveDistanceMeters()
            + rearRightMod.getDriveDistanceMeters())
            / 4.0
        );
    }

    /**
     * Returns the average velocity of each module to get an overall velocity of the robot.
     * 
     * @return The overall velocity of the robot in meters per second.
     */
    public double getAverageDriveVelocityMetersPerSecond() {
        return (
            (Math.abs(frontLeftMod.getCurrentVelocityMetersPerSecond())
            + Math.abs(frontRightMod.getCurrentVelocityMetersPerSecond())
            + Math.abs(rearLeftMod.getCurrentVelocityMetersPerSecond() )
            + Math.abs(rearRightMod.getCurrentVelocityMetersPerSecond()))
            / 4.0
        );
    }

    /**
     * Returns the average direction of each module to get an overall direction of travel of the robot.
     * 
     * @return The overall direction of travel of the robot
     */
    public Rotation2d getDirectionOfTravel() {
        return new Rotation2d(
            (frontLeftMod.getCanCoderAngle().plus(new Rotation2d(frontLeftMod.getCurrentVelocityMetersPerSecond() < 0.0 ? Math.PI : 0.0)).getRadians()
            + frontRightMod.getCanCoderAngle().plus(new Rotation2d(frontRightMod.getCurrentVelocityMetersPerSecond() < 0.0 ? Math.PI : 0.0)).getRadians()
            + rearLeftMod.getCanCoderAngle().plus(new Rotation2d(rearLeftMod.getCurrentVelocityMetersPerSecond() < 0.0 ? Math.PI : 0.0)).getRadians()
            + rearRightMod.getCanCoderAngle().plus(new Rotation2d(rearRightMod.getCurrentVelocityMetersPerSecond() < 0.0 ? Math.PI : 0.0)).getRadians()
            ) / 4.0
        );
    }

    /**
     * Returns the average velocity in the forward direction.
     * 
     * @return The velocity in the forward direction of the robot in meters per second.
     */
    public double getForwardVelocityMetersPerSecond() {
        double rel = Math.abs(getDirectionOfTravel().getDegrees() % 90.0);
        if(getDirectionOfTravel().getDegrees() > 90.0 || getDirectionOfTravel().getDegrees() < -90.0)
            rel -= 90.0;

        return getAverageDriveVelocityMetersPerSecond() * (Math.copySign((90.0 - rel), rel) / 90.0);
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
        rearLeftMod.setDriveCurrentLimit(amps);
        rearRightMod.setDriveCurrentLimit(amps);
    }
}