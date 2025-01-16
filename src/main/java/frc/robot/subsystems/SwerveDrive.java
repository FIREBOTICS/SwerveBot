package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

@Logged
public class SwerveDrive extends SubsystemBase {
    // Initialize new swerve module objects
    private final DataPortSwerveModule frontLeftMod = new DataPortSwerveModule(
        1,
        CANDevices.frontLeftDriveInverted,
        CANDevices.frontLeftSteerInverted,
        DriveConstants.frontLeftModuleOffset);

    private final DataPortSwerveModule frontRightMod = new DataPortSwerveModule(
        2,
        CANDevices.frontRightDriveInverted,
        CANDevices.frontRightSteerInverted,
        DriveConstants.frontRightModuleOffset);

    private final DataPortSwerveModule backLeftMod = new DataPortSwerveModule(
        3,
        CANDevices.backLeftDriveInverted,
        CANDevices.backLeftSteerInverted,
        DriveConstants.backLeftModuleOffset);

    private final DataPortSwerveModule backRightMod = new DataPortSwerveModule(
        4,
        CANDevices.backRightDriveInverted,
        CANDevices.backRightSteerInverted,  
        DriveConstants.backRightModuleOffset);

    private final AHRS navX = new AHRS(AHRS.NavXComType.kUSB1);

    private boolean isLocked = false;
    private boolean isFieldOriented = true;
    private String speedFactorKey = "Speed";
    private double speedFactor = Preferences.getDouble(speedFactorKey, 0.3);
    private boolean isTracking = false;
    // Odometry for the robot, measured in meters for linear motion and radians for
    // rotational motion
    // Takes in kinematics and robot angle for parameters
    @NotLogged
    private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d());

    public boolean isLocked() {
        return isLocked;
    }
    
    public boolean isFieldOriented() {
        return isFieldOriented;
    }
    private void setFieldOriented(boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
    }
    public Command setFieldOrientedCommand(boolean isFieldOriented) {
        return runOnce(() -> setFieldOriented(isFieldOriented));
    }
    public Command toggleFieldOrientedCommand() {
        return runOnce(() -> setFieldOriented(!isFieldOriented));
    }

    public double getSpeedFactor() {
        return speedFactor;
    }
    private void setSpeedFactor(double newSpeedFactor) {
        this.speedFactor = newSpeedFactor;
        Preferences.setDouble(speedFactorKey, speedFactor);
        System.out.println(speedFactor);
    }
    /**
     * Increase driving speed by percentage
     * @param difference Amount of speed to increase by
     */
    public Command speedUpCommand(double difference) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> setSpeedFactor(speedFactor+difference));
    }
    /**
     * Increase driving speed by percentage
     * @param difference Amount of speed to increase by
     */
    public Command slowDownCommand(double difference) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> setSpeedFactor(speedFactor-difference));
    }

    public boolean isTracking() {
        return isTracking;
    }
    public void setIsTracking(boolean isTracking) {
        this.isTracking = isTracking;
    }

    /**
     * Brings the drivetrain to a stop.
     * 
     * <p> Sets the drive power of each module to
     * zero while maintaining module headings.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0);
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
        return runOnce(this::lock);
    }
    
    /**
     * Sets the desired state for each swerve module.
     * 
     * <p> Each module uses PID and feedforward control (closed-loop)
     * to update speed, angle setpoints.
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
     * Sets the desired state for each swerve module, using open loop.
     * 
     * <p> Each module uses PID and feedforward control (open-loop)
     * to update speed, angle setpoints.
     * 
     * @param moduleStates the module states to set.
     */
    public void setModuleStatesAuto(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        backLeftMod.setDesiredState(moduleStates[2], true);
        backRightMod.setDesiredState(moduleStates[3], true);
    }

    /**
     * Sets modules to follow a specified Chassis Speed (for use in auto)
     * 
     * @param chassisSpeeds The {@link ChassisSpeeds} object to set the modules.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStatesAuto(DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
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
     * Returns the current pitch of the robot from the gyro.
     * 
     * @return The current pitch of the robot.
     */
    public double getPitchDegrees() {
        return navX.getPitch();
    }

    /**
     * Returns the current roll of the robot from the gyro.
     * 
     * @return The current roll of the robot.
     */
    public double getRollDegrees() {
        return navX.getRoll();
    }

    /**
     * Returns the current heading of the robot from the gyro.
     * 
     * @return The current heading of the robot.
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(navX.getYaw());
    }

    /**
     * Sets the gyro heading to zero.
     */
    public void resetHeading() {
        navX.zeroYaw();
    }

    /**
     * Sets the gyro heading to zero as a runOnce.
     */
    public Command resetHeadingCommand() {
        return runOnce(this::resetHeading);
    }

    /**
     * Resets the measured distance driven for each module.
     */
    public void resetDriveDistances() {
        frontLeftMod.resetDistance();
        frontRightMod.resetDistance();
        backLeftMod.resetDistance();
        backRightMod.resetDistance();
    }

    /**
     * @return The current estimated position of the robot on the field
     *         based on drive encoder and gyro readings.
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
                new Pose2d());
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(navX.getRotation2d(), getModulePositions(), pose);
    }

    public void setDriveCurrentLimit(int amps) {
        frontLeftMod.setDriveCurrentLimit(amps);
        frontRightMod.setDriveCurrentLimit(amps);
        backLeftMod.setDriveCurrentLimit(amps);
        backRightMod.setDriveCurrentLimit(amps);
    }

    public void setSteerCurrentLimit(int amps) {
        frontLeftMod.setSteerCurrentLimit(amps);
        frontRightMod.setSteerCurrentLimit(amps);
        backLeftMod.setSteerCurrentLimit(amps);
        backRightMod.setSteerCurrentLimit(amps);
    }

    public Command driveCommand(
        DoubleSupplier driveSupplier, 
        DoubleSupplier strafeSupplier, 
        DoubleSupplier rotSupplier) {
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
                        -rot
                    );
                }
        );
    }

    /**
     * Inputs drive values into the swerve drive base.
     * 
     * @param driveX The desired forward/backward lateral motion, in meters per second.
     * @param driveY The desired left/right lateral motion, in meters per second.
     * @param rotation The desired rotational motion, in radians per second.
     */
    public void drive(double driveX, double driveY, double rotation) {  
        if(driveX != 0.0 || driveY != 0.0 || rotation != 0.0) isLocked = false;
        
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
            rotation *= speedFactor; //make individually modifiable?

            // Represents the overall state of the drive base.
            ChassisSpeeds speeds =
                isFieldOriented
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveX, driveY, rotation, getHeading())
                    : new ChassisSpeeds(driveX, driveY, rotation);

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states.
            SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds); 
            
            // Makes sure the wheels don't try to spin faster than the maximum speed possible
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

            setModuleStates(states);
        }
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);

        /** EXPERIMENT WITH THIS INSTEAD
         * May be better because this will be open-loop, which is apparently for auto.
         * I don't understand this stuff enough
         */
        //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        //setChassisSpeeds(targetSpeeds);
        /* */
    }

    public Command encodersTestModeCommand() {
        return startEnd(
            () -> {
                frontLeftMod.isTestMode(true);
                frontRightMod.isTestMode(true);
                backLeftMod.isTestMode(true);
                backRightMod.isTestMode(true);
            },
            () -> {
                frontLeftMod.isTestMode(false);
                frontRightMod.isTestMode(false);
                backLeftMod.isTestMode(false);
                backRightMod.isTestMode(false);
            });
    }


    /**
     * Create a new Swerve Drive, including 4 modules and a navX
     */
    public SwerveDrive() {
        Preferences.initDouble(speedFactorKey, speedFactor);

        try {
            DriveConstants.PPRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            // PPRobotConfig = new RobotConfig(
            //   0,
            //   0,
            //   new ModuleConfig(0,
            //     0,0,
            //     DCMotor.getNEO(1),
            //     0,
            //     0),
            //   trackWidth
            // );
        }
        // Default PathPlanner config
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(1, 0, 0), // Translation PID constants
                        new PIDConstants(1, 0, 0) // Rotation PID constants
                ),
                DriveConstants.PPRobotConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Updates the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());
    }
}
