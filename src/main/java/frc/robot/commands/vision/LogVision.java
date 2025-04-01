package frc.robot.commands.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Vision.DetectedObject;

public class LogVision extends Command {
    private Supplier<DetectedObject> objectSupplier;
    public LogVision(Supplier<DetectedObject> objectSupplier){
        this.objectSupplier = objectSupplier;
    }

    @Override
    public void execute() {
        DetectedObject object = this.objectSupplier.get();
        if (object != null) {
            Logger.recordOutput("Vision/object_angle", (object.getAngle() + (Math.PI/2.0)) % Math.PI);
            Logger.recordOutput("Vision/object_distance", object.getDistance());
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
