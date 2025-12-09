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

public class AimAtGamePiece extends DefaultDriveCommand {
    private Supplier<DetectedObject> objectSupplier;
    private ObjectType targetType;
    private static int ticksSinceLastObject;
    private static DetectedObject cachedObject;

    public AimAtGamePiece(Drivetrain drive, BaseDriverConfig driver, Supplier<DetectedObject> objectSupplier, ObjectType targetType) {
        super(drive, driver);
        this.objectSupplier = objectSupplier;
        this.targetType = targetType;
    }

    @Override
    public void initialize() {
        cachedObject = null;
        ticksSinceLastObject = 0;
        super.initialize();
    }

    @Override
    protected void drive(ChassisSpeeds speeds) {

        if (!VisionConstants.OBJECT_DETECTION_ENABLED) {
            super.drive(speeds);
            return;
        }
        DetectedObject object = objectSupplier.get();

        if (object == null || object.type != targetType) {
            if (ticksSinceLastObject <= VisionConstants.MAX_EMPTY_TICKS && cachedObject != null) {
                object = cachedObject;
            } else {
                super.drive(speeds);
                return;
            }
            ticksSinceLastObject++;
        } else {
            ticksSinceLastObject = 0;
            cachedObject = object;
        }

        // System.out.println("curangle " + swerve.getPose().getRotation().getDegrees());
        // System.out.println("objangle " + object.getAngle());
        swerve.driveHeading(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                MathUtil.angleModulus(object.getAngle() + Math.PI / 2),
                true);
    }
}

