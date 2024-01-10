package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class DecreaseSpdCmd extends Command {

    private final SwerveSys swerveSys;


    private final double difference;

    public DecreaseSpdCmd(double difference, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.difference = difference;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
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
