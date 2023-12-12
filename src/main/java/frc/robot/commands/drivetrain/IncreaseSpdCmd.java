package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class IncreaseSpdCmd extends CommandBase {

    private final SwerveSys swerveSys;

    private final double difference;

    public IncreaseSpdCmd(double difference, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.difference = difference;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveSys.increaseSpeedFactor(difference);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
