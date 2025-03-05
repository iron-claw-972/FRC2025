package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs{
        public double positionDeg = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
}
