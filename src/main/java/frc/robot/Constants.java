package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class CANDevices {
        public static final int powerDistributionHubId = 0;

        /* Swerve Module IDs
         * 1 = Front Left
         * 2 = Front Right
         * 3 = Back Left
         * 4 = Back Right
         */

        /* Swerve Motor IDs
         * 1 = Drive
         * 2 = Turn
         */

        /* Swerve Encoder ID
         * 3
        */
        public static final int frontLeftDriveMotorId = 11;
        public static final int frontLeftSteerMotorId = 12;
        public static final int frontLeftSteerEncoderId = 13;

        public static final int frontRightDriveMotorId = 21;
        public static final int frontRightSteerMotorId = 22;
        public static final int frontRightSteerEncoderId = 23;

        public static final int rearLeftDriveMotorId = 31;
        public static final int rearLeftSteerMotorId = 32;
        public static final int rearLeftSteerEncoderId = 33;

        public static final int rearRightDriveMotorId = 41;
        public static final int rearRightSteerMotorId = 42;
        public static final int rearRightSteerEncoderId = 43;

        public static final boolean frontLeftReversed = false;
        public static final boolean frontRightReversed = true;
        public static final boolean rearLeftReversed = false;
        public static final boolean rearRightReversed = true;
    }

    public static final class ControllerConstants {

        public static final int driverGamepadPort = 0;

        public static final double joystickDeadband = 0.15;

        public static final double triggerPressedThreshhold = 0.25;
    }
    
    public static final class DriveConstants {

        /**
         * The track width from wheel center to wheel center.
         */
        public static final double trackWidth = Units.inchesToMeters(24.375);

        /**
         * The track length from wheel center to wheel center.
         */
        public static final double wheelBase = Units.inchesToMeters(24.375);

        /**
         * The SwerveDriveKinematics used for control and odometry.
         */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
            );

        /**
         * The gear reduction from the drive motor to the wheel.
         */
        public static final double driveMtrGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        /**
         * The gear reduction from the steer motor to the wheel.
         */
        public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelRadiusMeters = 0.0508;
        public static final double wheelCircumferenceMeters = 2 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60;

        public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;

        public static final double kFreeMetersPerSecond = 5600 * driveMetersPerSecPerRPM;

        public static final double steerMtrMaxSpeedRadPerSec = 2.0;
        public static final double steerMtrMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = 5.0;

        /**
         * The rate the robot will spin with full Rot command.
         */
        public static final double maxTurnRateRadiansPerSec = 2 * Math.PI;

        public static final double frontLeftModOffset  = Units.degreesToRadians(33.35); //221
        public static final double frontRightModOffset = Units.degreesToRadians(158.25); //162.5
        public static final double rearLeftModOffset   = Units.degreesToRadians(129.6); //125.4
        public static final double rearRightModOffset  = Units.degreesToRadians(325.5); //323.5

        public static final int driveCurrentLimitAmps = 35;
        
        public static final double drivekP = 0.1; //Benjy Changed From 0.005 to 0.1


        public static final double steerkP = 0.37431;
        // public static final double steerkD = 0.27186;

        // public static final double steerkP = 0.1;
        public static final double steerkI = 0;
        public static final double steerkD = 0;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 2.44;
        public static final double kaVoltSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 3.25;
        public static final double maxAccelMetersPerSecondSq = 1.625;

        public static final double drivekP = 12.8;
        public static final double drivekD = 0.085;

        public static final double rotkP = 1.27;
        public static final double rotkD = 0.5;

        public final static TrajectoryConfig config = 
            new TrajectoryConfig(
                AutoConstants.maxVelMetersPerSec, 
                AutoConstants.maxAccelMetersPerSecondSq
            )
            .setKinematics(DriveConstants.kinematics
        );

        public static final PIDController driveController = new PIDController(
            drivekP,
            0,
            drivekD
        );

        public static final PIDController rotController = constructRotController();
        private static PIDController constructRotController() {
            PIDController rotController = new PIDController(
                rotkP,
                0,
                rotkD
            );

            // Sets wraparound from 0 to 360.
            rotController.enableContinuousInput(0, 360);
            
            return rotController;
        }
    }
}
