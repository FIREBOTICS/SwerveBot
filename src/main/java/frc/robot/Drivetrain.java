// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 1.3 * Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(
            1,
            CANDevices.frontLeftDriveInverted,
            CANDevices.frontLeftSteerInverted,
            DriveConstants.frontLeftModuleOffset);
  private final SwerveModule m_frontRight = new SwerveModule(
            2,
            CANDevices.frontRightDriveInverted,
            CANDevices.frontRightSteerInverted,
            DriveConstants.frontRightModuleOffset);
  private final SwerveModule m_backLeft = new SwerveModule(
            3,
            CANDevices.backLeftDriveInverted,
            CANDevices.backLeftSteerInverted,
            DriveConstants.backLeftModuleOffset);
  private final SwerveModule m_backRight = new SwerveModule(
            4,
            CANDevices.backRightDriveInverted,
            CANDevices.backRightSteerInverted,
            DriveConstants.backRightModuleOffset);

  private final AHRS navX = new AHRS(SerialPort.Port.kUSB);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          navX.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    navX.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, navX.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0], false);
    m_frontRight.setDesiredState(swerveModuleStates[1], false);
    m_backLeft.setDesiredState(swerveModuleStates[2], false);
    m_backRight.setDesiredState(swerveModuleStates[3], false);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        navX.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /** Resets the field-oriented zero of the robot to current heading */
  public void resetHeading() {
    navX.reset();
  }


  public void displayAngles() {
    SmartDashboard.putNumber("Front Left Rotation", 360-m_frontLeft.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Front Right Rotation", 360-m_frontRight.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Back Left Rotation", 360-m_backLeft.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Back Right Rotation", 360-m_backRight.getCanCoderAngle().getDegrees());

    SmartDashboard.putNumber("Angle", navX.getAngle());
  }

}
