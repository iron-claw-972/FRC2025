package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double measuredPosition = 0.0;
        public double velocity = 0.0;
        public double currentAmps = 0.0;
    }
}
