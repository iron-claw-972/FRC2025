package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double measuredAngle = 0.0;
        public double currentAmps = 0.0;
    }

    public void updateInputs();
}
