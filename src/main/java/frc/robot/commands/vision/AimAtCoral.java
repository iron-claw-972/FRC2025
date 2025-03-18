package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.drive_comm.DefaultDriveCommand;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.DetectedObject.ObjectType;

public class AimAtCoral extends DefaultDriveCommand {
    private Supplier<DetectedObject> objectSupplier;
    public AimAtCoral(Drivetrain drive, BaseDriverConfig driver, Supplier<DetectedObject> objectSupplier){
        super(drive, driver);
        this.objectSupplier = objectSupplier;
    }

    @Override
    protected void drive(ChassisSpeeds speeds){
        if(!VisionConstants.OBJECT_DETECTION_ENABLED){
            super.drive(speeds);
            return;
        }
        DetectedObject object = objectSupplier.get();
        if(object == null || object.type != ObjectType.CORAL){
            super.drive(speeds);
            return;
        }
        swerve.driveHeading(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            MathUtil.angleModulus(object.getAngle()+Math.PI/2),
            true);
}
}
