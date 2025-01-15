package frc.robot.loggers;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(AbsoluteEncoder.class)
public class SparkAbsoluteEncoderLogger extends ClassSpecificLogger<AbsoluteEncoder> {
    public SparkAbsoluteEncoderLogger() {
        super(AbsoluteEncoder.class);
    }
    
    public void update(EpilogueBackend backend, AbsoluteEncoder absoluteEncoder) {
        backend.log("Position (Default: Rotations)", absoluteEncoder.getPosition());
        backend.log("Velocity (Default: RPM)", absoluteEncoder.getVelocity());
    }
}
