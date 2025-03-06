package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;


import edu.wpi.first.math.geometry.Pose2d;

public interface DrivetrainIO {
    @AutoLog
    public static class DrivetrainIOInputs {
        public double speedX = 0.0;
        public double speedY = 0.0;
        public double speed = 0.0;
        public double speedRot = 0.0;
        public Pose2d pose2d = new Pose2d();
    }
}
