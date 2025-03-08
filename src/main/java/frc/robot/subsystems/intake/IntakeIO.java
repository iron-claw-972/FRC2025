package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean hasCoral = false;
        public boolean atSetpoint = false;
        public double rollerVelocity = 0.0;
        public double measuredPivotPosition = 0.0;
    }
}
