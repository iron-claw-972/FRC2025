package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.MathUtils;

/**
 * Abstract class for different controller types.
 */
public abstract class BaseDriverConfig {

    private final Drivetrain drive;

    private double previousHeading = 0;

    private final DynamicSlewRateLimiter headingLimiter = new DynamicSlewRateLimiter(Constants.HEADING_SLEWRATE);

    /**
     * @param drive               the drivetrain instance
     * @param controllerTab       the shuffleboard controller tab
     * @param shuffleboardUpdates whether to update the shuffleboard
     */
    public BaseDriverConfig(Drivetrain drive) {
        headingLimiter.setContinuousLimits(-Math.PI, Math.PI);
        headingLimiter.enableContinuous(true);
        this.drive = drive;
    }

    public double getForwardTranslation() {
        double forward = getRawForwardTranslation();
        return forward * DriveConstants.MAX_SPEED * Math.min(1,RobotController.getBatteryVoltage()/12) * MathUtil.applyDeadband(Math.sqrt(forward*forward + Math.pow(getRawSideTranslation(), 2)), Constants.TRANSLATIONAL_DEADBAND);
    }

    public double getSideTranslation() {
        double side = getRawSideTranslation();
        return side * DriveConstants.MAX_SPEED * Math.min(1,RobotController.getBatteryVoltage()/12) * MathUtil.applyDeadband(Math.sqrt(side*side + Math.pow(getRawForwardTranslation(), 2)), Constants.TRANSLATIONAL_DEADBAND);
    }

    public double getRotation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawRotation(), Constants.ROTATION_DEADBAND), 2)
                * DriveConstants.MAX_ANGULAR_SPEED * Math.min(1, RobotController.getBatteryVoltage()/12);
    }

    public double getHeading() {
        if (getRawHeadingMagnitude() <= Constants.HEADING_DEADBAND)
            return headingLimiter.calculate(previousHeading, 1e-6);
        previousHeading = headingLimiter.calculate(getRawHeadingAngle(),
                MathUtils.expoMS(getRawHeadingMagnitude(), 2));
        return previousHeading;
    }

    protected Drivetrain getDrivetrain() {
        return drive;
    }

    /**
     * Configures the controls for the controller.
     */
    public abstract void configureControls();

    public abstract double getRawSideTranslation();

    public abstract double getRawForwardTranslation();

    public abstract double getRawRotation();

    public abstract double getRawHeadingAngle();

    public abstract double getRawHeadingMagnitude();

    public abstract boolean getIsSlowMode();

    public abstract boolean getIsAlign();

}