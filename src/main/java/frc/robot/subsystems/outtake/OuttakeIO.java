package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    public static class OuttakeIOIntakes {
        public boolean coralLoaded = false;
        public boolean coralEjecting = false;
        public double motorVelocity = 0.0;
    }
}
