// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  private boolean isFieldOriented = true;

  // Initialize subsystems.
  private final SwerveDrive swerveDrive = new SwerveDrive();

  // Initialize auto selector.
  SendableChooser<Command> autoSelector = new SendableChooser<Command>();

  // Create Field2d object to put on Dashboard
  private final Field2d field = new Field2d();

  @NotLogged
  private final CommandXboxController driverController =
    new CommandXboxController(ControllerConstants.driverControllerPort);
  @NotLogged
  private final CommandXboxController codriverController =
    new CommandXboxController(ControllerConstants.codriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("auto selector", autoSelector);

    SmartDashboard.putData(swerveDrive);

    SmartDashboard.putData(field);
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });
    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger (java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
   private void configureBindings() {
    // Schedule `lock` when the Xbox controller's left trigger is beyond the threshold,
    // cancelling on release.
    driverController.leftTrigger(ControllerConstants.triggerPressedThreshhold).whileTrue(swerveDrive.lockCommand());
    driverController.x().onTrue(swerveDrive.speedUpCommand(0.1));
    driverController.a().onTrue(swerveDrive.slowDownCommand(0.1));
    swerveDrive.setDefaultCommand(
        swerveDrive.driveCommand(
          () -> -deadband(driverController.getLeftY()),
          () -> -deadband(driverController.getLeftX()),
          () -> -deadband(driverController.getRightX()),
          () -> isFieldOriented
          )
    );

    RobotModeTriggers.test().whileTrue(swerveDrive.encodersTestModeCommand());

    // SmartDashboard.putData("swerve", swerveDrive);
  }

  /**
   * Deadbands inputs to eliminate tiny unwanted values from the joysticks or gamepad sticks.

   * <p> If the distance between the input and zero is less than the deadband amount, the output will be zero.
   * Otherwise, the value will not change.
   * 
   * @param value The controller value to deadband.
   * @return The deadbanded controller value.
   */
  public double deadband(double value) {
    if (Math.abs(value) < ControllerConstants.joystickDeadband)
        return 0.0;
    return value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public Command toggleFieldOrientedCommand() {
    return Commands.runOnce(
      () -> isFieldOriented = !isFieldOriented);
  }

}
