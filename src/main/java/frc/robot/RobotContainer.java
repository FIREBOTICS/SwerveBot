package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drivetrain.IncreaseSpdCmd;
import frc.robot.commands.drivetrain.DecreaseSpdCmd;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.commands.drivetrain.ResetHeadingCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();

    // Initialize joysticks.
    private final XboxController driverController = new XboxController(ControllerConstants.driverGamepadPort);

    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);

    private final Trigger driverLeftTriggerBtn =
        new Trigger(() -> driverController.getLeftTriggerAxis() > ControllerConstants.triggerPressedThreshhold);

    private final Trigger driverSpeedUpButton = 
        new Trigger(() -> driverController.getXButtonPressed());

    private final Trigger driverSlowDownButton = 
        new Trigger(() -> driverController.getAButtonPressed());

    // Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        SmartDashboard.putData("auto selector", autoSelector);

        configDriverBindings();
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(
                new SwerveDriveCmd(
                () -> deadband(driverController.getLeftY()),
                () -> deadband(driverController.getLeftX()),
                () -> deadband(driverController.getRightX()),
                true, //Switch to False for Chairbot Mode
                swerveSys
            )
        );
        driverMenuBtn.onTrue(new ResetHeadingCmd(swerveSys));

        driverLeftTriggerBtn.whileTrue(new LockCmd(swerveSys));

        driverSpeedUpButton.onTrue(new IncreaseSpdCmd(
            0.1,
            swerveSys)
        );
        driverSlowDownButton.onTrue(new DecreaseSpdCmd(
            0.1, 
            swerveSys)
        );
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    /**
     * Deadbands inputs to eliminate tiny unwanted values from the joysticks or gamepad sticks.
     * <p>If the distance between the input and zero is less than the deadband amount, the output will be zero.
     * Otherwise, the value will not change.
     * 
     * @param input The controller value to deadband.
     * have different deadbands.
     * @return The deadbanded controller value.
     */
    public double deadband(double value) {

        if (Math.abs(value) < ControllerConstants.joystickDeadband)
            return 0.0;
        
        return value;
    }

    public void updateInterface() {
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m//s", swerveSys.getAverageDriveVelocityMetersPerSecond());
    }
}
