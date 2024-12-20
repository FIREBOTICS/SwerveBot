// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double steerkP = 0.3;
    public static final double steerkI = 0;
    public static final double steerkD = 0.01;

    public static final int driveCurrentLimitAmps = 30;
    public static final int steerCurrentLimitAmps = 20;

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
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left
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

    public static final double maxDriveSpeedMetersPerSec = 0.2; //Max Speed
    
    public static final double frontLeftModuleOffset  = Units.degreesToRadians(130.5); //122
    public static final double frontRightModuleOffset = Units.degreesToRadians(72.5); //-290
    public static final double backLeftModuleOffset   = Units.degreesToRadians(29.4); //34.3
    public static final double backRightModuleOffset  = Units.degreesToRadians(237.6); //-307 + 175.6
  

    /**
     * The rate the robot will spin with full Rot command.
     */
    public static final double maxTurnRateRadiansPerSec = 2 * Math.PI;

    public static final double ksVolts = 0.667;
    public static final double kvVoltSecsPerMeter = 2.44;
    public static final double kaVoltSecsPerMeterSq = 0.0;
    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter,
        kaVoltSecsPerMeterSq);

  }

  public static class SubsystemConstants {
    public static final double ampShotSpeed = 0.4;
    public static final double speakerShotSpeed = 1;
    public static final double trapShotSpeed = 0.7; // CALIBRATE LATER

    public static final double intakeMotorSpeed = 0.9;
    public static final double loaderMotorSpeed = 0.3;
    public static final double bumperMotorRejectSpeed = 0.7; // these should have opposite signs:
    public static final double bumperMotorIntakeSpeed = 0.7;

    public static final double climberMotorSpeed = 0.5;
  }

  public static final class CANDevices {
    public static final int powerDistributionHubId = 0;

    /*
     * Swerve Module IDs
     * 1 = Front Left
     * 2 = Front Right
     * 3 = Back Left
     * 4 = Back Right
     */

    /*
     * Swerve Motor IDs
     * 1 = Drive
     * 2 = Turn
     */

    /*
     * Swerve Encoder ID
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

    public static final boolean frontLeftDriveInverted = false;
    public static final boolean frontRightDriveInverted = true;
    public static final boolean backLeftDriveInverted = false;
    public static final boolean backRightDriveInverted = false;

    public static final boolean frontLeftSteerInverted = false;
    public static final boolean frontRightSteerInverted = false;
    public static final boolean backLeftSteerInverted = true;
    public static final boolean backRightSteerInverted = true;
  }

  public static final class ControllerConstants {
    public static final int driverControllerPort = 0;
    public static final int codriverControllerPort = 1;

    public static final double joystickDeadband = 0.15;
    public static final double triggerPressedThreshhold = 0.25;
  }

}
