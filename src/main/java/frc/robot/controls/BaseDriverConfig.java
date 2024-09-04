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
public abstract class BaseDriverConfig {

    private final Drivetrain drive;

    // Some of these are not currently used, but we might want them later
    @SuppressWarnings("unused")
    private double translationalSensitivity = Constants.TRANSLATIONAL_SENSITIVITY;
    @SuppressWarnings("unused")
    private double translationalExpo = Constants.TRANSLATIONAL_EXPO;
    @SuppressWarnings("unused")
    private double translationalDeadband = Constants.TRANSLATIONAL_DEADBAND;
    private double translationalSlewrate = Constants.TRANSLATIONAL_SLEWRATE;

    @SuppressWarnings("unused")
    private double rotationSensitivity = Constants.ROTATION_SENSITIVITY;
    @SuppressWarnings("unused")
    private double rotationExpo = Constants.ROTATION_EXPO;
    @SuppressWarnings("unused")
    private double rotationDeadband = Constants.ROTATION_DEADBAND;
    private double rotationSlewrate = Constants.ROTATION_SLEWRATE;

    private double headingSensitivity = Constants.HEADING_SENSITIVITY;
    private double headingExpo = Constants.HEADING_EXPO;
    private double headingDeadband = Constants.HEADING_DEADBAND;
    private double previousHeading = 0;

    @SuppressWarnings("unused")
    private final DynamicSlewRateLimiter xSpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
    @SuppressWarnings("unused")
    private final DynamicSlewRateLimiter ySpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
    @SuppressWarnings("unused")
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
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawForwardTranslation(), Constants.DEADBAND), 2)
                * DriveConstants.kMaxSpeed * 1;
    }

    public double getSideTranslation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawSideTranslation(), Constants.DEADBAND), 2)
                * DriveConstants.kMaxSpeed * 1;
    }

    public double getRotation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawRotation(), Constants.HEADINGDEADBAND), 2)
                * DriveConstants.kMaxAngularSpeed * 1;
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