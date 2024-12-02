package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.MathUtils;

/**
 * Abstract class for different controller types.
 */
@SuppressWarnings("unused")
public abstract class BaseDriverConfig {

    private final Drivetrain drive;

    // Some of these are not currently used, but we might want them later
    private double translationalSensitivity = Constants.TRANSLATIONAL_SENSITIVITY;
    private double translationalExpo = Constants.TRANSLATIONAL_EXPO;
    private double translationalDeadband = Constants.TRANSLATIONAL_DEADBAND;
    private double translationalSlewrate = Constants.TRANSLATIONAL_SLEWRATE;

    private double rotationSensitivity = Constants.ROTATION_SENSITIVITY;
    private double rotationExpo = Constants.ROTATION_EXPO;
    private double rotationDeadband = Constants.ROTATION_DEADBAND;
    private double rotationSlewrate = Constants.ROTATION_SLEWRATE;

    private double headingSensitivity = Constants.HEADING_SENSITIVITY;
    private double headingExpo = Constants.HEADING_EXPO;
    private double headingDeadband = Constants.HEADING_DEADBAND;
    private double previousHeading = 0;

    private final DynamicSlewRateLimiter xSpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
    private final DynamicSlewRateLimiter ySpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
    private final DynamicSlewRateLimiter rotLimiter = new DynamicSlewRateLimiter(rotationSlewrate);
    private final DynamicSlewRateLimiter headingLimiter = new DynamicSlewRateLimiter(headingSensitivity);

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
        return forward * DriveConstants.kMaxSpeed * MathUtil.applyDeadband(Math.sqrt(forward*forward + Math.pow(getRawSideTranslation(), 2)), Constants.TRANSLATIONAL_DEADBAND);
    }

    public double getSideTranslation() {
        double side = getRawSideTranslation();
        return side * DriveConstants.kMaxSpeed * MathUtil.applyDeadband(Math.sqrt(side*side + Math.pow(getRawForwardTranslation(), 2)), Constants.TRANSLATIONAL_DEADBAND);
    }

    public double getRotation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawRotation(), Constants.ROTATION_DEADBAND), 2)
                * DriveConstants.kMaxAngularSpeed;
    }

    public double getHeading() {
        if (getRawHeadingMagnitude() <= headingDeadband)
            return headingLimiter.calculate(previousHeading, 1e-6);
        previousHeading = headingLimiter.calculate(getRawHeadingAngle(),
                MathUtils.expoMS(getRawHeadingMagnitude(), headingExpo) * headingSensitivity);
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