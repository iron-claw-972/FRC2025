package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    public static class OuttakeIOIntakes {
        public double motorVelocity = 0.0;
        public int proximity = 0;
    }
}
