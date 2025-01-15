package frc.robot.loggers;

import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Canandmag.class)
public class CanandmagLogger extends ClassSpecificLogger<Canandmag> {
    public CanandmagLogger() {
        super(Canandmag.class);
    }

    @Override
    public void update(EpilogueBackend backend, Canandmag canandmag) {
        /* It's quite possible that many of these are values we never use. However, why not log them? */
        backend.log("Absolute Position (Rotation)", canandmag.getAbsPosition());
        backend.log("Relative Position (Rotations)", canandmag.getPosition());
        backend.log("Velocity (RPS)", canandmag.getVelocity());
        backend.log("Temperature (C)", canandmag.getTemperature());
        backend.log("Encoder is connected", canandmag.isConnected());
        backend.log("Magnet is in range", canandmag.magnetInRange());
    }
    
}
