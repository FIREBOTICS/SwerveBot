package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class DecreaseSpdCmd extends CommandBase {

    private final SwerveSys swerveSys;


    private final double difference;

    public DecreaseSpdCmd(double difference, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.difference = difference;
    }

    @Override
    public void initialize() {
        System.out.println("Hello there1");
    }

    @Override
    public void execute() {
        System.out.println("Hello there2");
        swerveSys.decreaseSpeedFactor(difference);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
