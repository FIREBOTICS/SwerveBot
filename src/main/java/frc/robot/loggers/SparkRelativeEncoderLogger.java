package frc.robot.loggers;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(RelativeEncoder.class)
public class SparkRelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {
    public SparkRelativeEncoderLogger() {
        super(RelativeEncoder.class);
    }
    
    public void update(EpilogueBackend backend, RelativeEncoder relativeEncoder) {
        backend.log("Position (Default: Rotations)", relativeEncoder.getPosition());
        backend.log("Velocity (Default: RPM)", relativeEncoder.getVelocity());
    }
}
