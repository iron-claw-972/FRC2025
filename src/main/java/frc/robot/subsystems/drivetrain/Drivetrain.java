package frc.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.PhoenixOdometryThread;
import frc.robot.util.SwerveModulePose;
import frc.robot.util.SwerveStuff.SwerveSetpoint;
import frc.robot.util.SwerveStuff.SwerveSetpointGenerator;
import frc.robot.util.Vision.Vision;

/**
 * Represents a swerve drive style drivetrain.
 * <p>
 * Module IDs are:
 * 1: Front left
 * 2: Front right
 * 3: Back left
 * 4: Back right
 */
public class Drivetrain extends SubsystemBase {

    protected final Module[] modules;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public static Lock odometryLock = new ReentrantLock();

    private SwerveSetpoint currentSetpoint =
    new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        });
    // Odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    // Vision
    private final Vision vision;


    // PID Controllers for chassis movement
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    // If vision is enabled for drivetrain odometry updating
    // DO NOT CHANGE THIS HERE TO DISABLE VISION, change VisionConstants.ENABLED instead
    private boolean visionEnabled = true;

    // Disables vision for the first few seconds after deploying
    private Timer visionEnableTimer = new Timer();

    // If the robot should algin to the angle
    private boolean isAlign = false;
    // Angle to align to, can be null
    private Double alignAngle = null;
    // used for drift control
    private double currentHeading = 0;
    // used for drift control
    private boolean drive_turning = false;

    private SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();

    // The pose supplier to drive to
    private Supplier<Pose2d> desiredPoSupplier = ()->null;

    private SwerveModulePose modulePoses;

    // The previous pose to reset to if the current pose gets too far off the field
    private Pose2d prevPose = new Pose2d();

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];;

    private boolean slipped = false;

    private double previousAngularVelocity = 0;

    private double centerOfMassHeight = 0;

    private Rotation2d rawGyroRotation = new Rotation2d();


    /**
     * Creates a new Swerve Style Drivetrain.
     */
    public Drivetrain(Vision vision, GyroIO gyroIO) {
        this.vision = vision;

        modules = new Module[4];
        this.gyroIO = gyroIO;
        ModuleConstants[] constants = Arrays.copyOfRange(ModuleConstants.values(), 0, 4);

        if(RobotBase.isReal()){
            Arrays.stream(constants).forEach(moduleConstants -> {
                modules[moduleConstants.ordinal()] = new Module(moduleConstants);
            });
        }else{
            Arrays.stream(constants).forEach(moduleConstants -> {
                modules[moduleConstants.ordinal()] = new ModuleSim(moduleConstants);
            });
        }

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        gyroIO.updateInputs(gyroInputs);
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.KINEMATICS,
                gyroInputs.yawPosition,
                updateModulePositions(),
                new Pose2d(),
                // Defaults, except trust pigeon more
                VecBuilder.fill(0.1, 0.1, 0),
                VisionConstants.VISION_STD_DEVS
        );
       poseEstimator.setVisionMeasurementStdDevs(VisionConstants.VISION_STD_DEVS);
        
        // initialize PID controllers
        xController = new PIDController(DriveConstants.TRANSLATIONAL_P, 0, DriveConstants.TRANSLATIONAL_D);
        yController = new PIDController(DriveConstants.TRANSLATIONAL_P, 0, DriveConstants.TRANSLATIONAL_D);
        rotationController = new PIDController(DriveConstants.HEADING_P, 0, DriveConstants.HEADING_D);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(0.25), Units.degreesToRadians(0.25));

        PhoenixOdometryThread.getInstance().start();

        modulePoses = new SwerveModulePose(this, DriveConstants.MODULE_LOCATIONS);
        

        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });

        //PPLibTelemetry.enableCompetitionMode();
    }

    public void close() {
        // close each of the modules
        for (int i = 0; i < modules.length; i++) {
            modules[i].close();
        }
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();
            // Update odometry
        double[] sampleTimestamps =
            gyroInputs.odometryYawTimestamps; // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            } 
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.odometryYawPositions[i];
            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
        Logger.recordOutput("Odometry/module poses", modulePoses.getModulePoses());
        updateOdometryVision();
    }

    // DRIVE
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        speed of the robot in the x direction (forward) in m/s
     * @param ySpeed        speed of the robot in the y direction (sideways) in m/s
     * @param rot           angular rate of the robot in rad/s
     * @param fieldRelative whether the provided x and y speeds are relative to the field
     * @param isOpenLoop    whether to use velocity control for the drive motors
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        // rot = headingControl(rot, xSpeed, ySpeed);
        ChassisSpeeds speeds = ChassisSpeeds.discretize(xSpeed, ySpeed, rot, Constants.LOOP_TIME);
        if(fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        }
        setChassisSpeeds(speeds, isOpenLoop);
    }

    /**
     * Drives the robot using the provided x speed, y speed, and positional heading.
     *
     * @param xSpeed        speed of the robot in the x direction (forward)
     * @param ySpeed        speed of the robot in the y direction (sideways)
     * @param heading       target heading of the robot in radians
     * @param fieldRelative whether the provided x and y speeds are relative to the field
     */
    public void driveHeading(double xSpeed, double ySpeed, double heading, boolean fieldRelative) {
        double rot = rotationController.calculate(getYaw().getRadians(), heading);
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        if(fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        }
        setChassisSpeeds(speeds, false);
    }

    /**
     * Runs the PID controllers with the provided x, y, and rot values. Then, calls {@link #drive(double, double, double, boolean, boolean)} using the PID outputs.
     * This is based on the odometry of the chassis.
     *
     * @param x   the position to move to in the x, in meters
     * @param y   the position to move to in the y, in meters
     * @param rot the angle to move to, in radians
     */
    public void driveWithPID(double x, double y, double rot) {
        Pose2d pose = getPose();
        double xSpeed = xController.calculate(pose.getX(), x);
        double ySpeed = yController.calculate(pose.getY(), y);
        double rotRadians = rotationController.calculate(pose.getRotation().getRadians(), rot);
        drive(xSpeed, ySpeed, rotRadians, true, false);
    }

    /**
     * Updates odometry using vision
     */
    public void updateOdometryVision() {
        // Start the timer if it hasn't started yet
        visionEnableTimer.start();

        // Update the swerve module poses
        modulePoses.update();

        if(modulePoses.slipped()){
            slipped = true;
        }

        Pose2d pose2 = getPose();

        // Even if vision is disabled, it should still update inputs
        // This prevents it from storing a lot of unread results, and it could be useful for replays
        vision.updateInputs();

        if(VisionConstants.ENABLED){
            if(vision != null && visionEnabled && visionEnableTimer.hasElapsed(5)){
                vision.updateOdometry(poseEstimator, time->getPoseAt(time).getRotation().getRadians(), slipped);

                if(vision.canSeeTag()){
                    slipped = false;
                    modulePoses.reset();
                }
            }
        }

        Pose2d pose3 = getPose();
        
        // Reset the pose to a position on the field if it is too far off the field
        // This uses nearField() instead of onField() so we don't reset the odometry when the wheels slip near the edge of the field
        // This is meant for poses that are caused by errors
        if(!Vision.nearField(prevPose)){
            // If the pose at the beginning of the method is off the field, reset to a position in the middle of the field
            // Use the rotation of the pose after updating odometry so the yaw is right
            prevPose = new Pose2d(FieldConstants.FIELD_LENGTH/2, FieldConstants.FIELD_WIDTH/2, pose2.getRotation());
            resetOdometry(prevPose);
        }else if(!Vision.nearField(pose2)){
            // if the drivetrain pose is off the field, reset our odometry to the pose before(this is the right pose)
            // Keep the rotation from pose2 so yaw is correct for driver
            prevPose = new Pose2d(prevPose.getTranslation(), pose2.getRotation());
            resetOdometry(prevPose);
        }else if(!Vision.nearField(pose3)){
            //if our vision+drivetrain odometry isn't near the field, reset our odometry to the pose before(this is the right pose)
            resetOdometry(pose2);
            prevPose = pose2;
        }else{
            // Set the previous pose to the current pose if we need to return to that
            prevPose = pose3;
        }

        // if (Robot.isSimulation()) {
        //     pigeon.getSimState().addYaw(
        //             +Units.radiansToDegrees(currentSetpoint.chassisSpeeds().omegaRadiansPerSecond * Constants.LOOP_TIME));
        // }
    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        Arrays.stream(modules).forEach(Module::stop);
    }


    // GETTERS AND SETTERS

    /**
     * Sets the desired states for all swerve modules.
     *
     * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
        // makes sure speeds of modules don't exceed maximum allowed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /**
     * Sets the chassis speeds of the robot.
     *
     * @param chassisSpeeds the target chassis speeds
     * @param isOpenLoop    if open loop control should be used for the drive velocity
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {

        if(DriveConstants.USE_ACTUAL_SPEED){
            SwerveSetpoint currentState = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
            currentSetpoint = setpointGenerator.generateSetpoint(
                DriveConstants.MODULE_LIMITS,
                centerOfMassHeight,
                currentState, chassisSpeeds,
                Constants.LOOP_TIME);
        }else{
            currentSetpoint = setpointGenerator.generateSetpoint(
                DriveConstants.MODULE_LIMITS,
                centerOfMassHeight,
                currentSetpoint, chassisSpeeds,
                Constants.LOOP_TIME);
        }

        SwerveModuleState[] swerveModuleStates = currentSetpoint.moduleStates();
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void setDriveVoltages(Voltage voltage){
        for (int i = 0; i<modules.length;i++){
            modules[i].setDriveVoltage(voltage);
        }
    }

    public void setAngleMotors(Rotation2d[] angles){
        for (int i = 0;i<modules.length;i++){
            modules[i].setAngle(angles[i]);
        }
    }

    /**
     * Returns the angular rate from the pigeon.
     *
     * @param id 0 for x, 1 for y, 2 for z
     * @return the rate in rads/s from the pigeon
     */
    public double getAngularRate(int id) {
        //double speed = 0;
        // switch(id){
        //     case 0:
        //         speed = gyroInputs..getAngularVelocityXWorld().getValueAsDouble();
        //         break;
        //     case 1:
        //         speed = pigeon.getAngularVelocityYWorld().getValueAsDouble();
        //         break;
        //     case 2:
        //         speed = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        //         break;
        // }
        // outputs in deg/s, so convert to rad/s
        return gyroInputs.yawVelocityRadPerSec;
    }


    /**
     * Updates and returns the array of SwerveModulePositions, which store the distance travleled by the drive and the steer angle.
     *
     * @return An array of all swerve module positions
     */
    private SwerveModulePosition[] updateModulePositions() {
        return modulePositions = Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Gets an array of SwerveModulePositions, which store the distance travleled by the drive and the steer angle.
     *
     * @return An array of all swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return modulePositions;
    }

    /**
     * Enables or disables the state deadband for all swerve modules.
     * The state deadband determines if the robot will stop drive and steer motors when inputted drive velocity is low.
     * It should be enabled for all regular driving, to prevent releasing the controls from setting the angles.
     */
    public void setStateDeadband(boolean stateDeadBand) {
        Arrays.stream(modules).forEach(module -> module.setStateDeadband(stateDeadBand));
    }
    public void setOptimized(boolean optimized) {
        Arrays.stream(modules).forEach(module -> module.setOptimize(optimized));
    }

    public void setVisionEnabled(boolean enabled){
        visionEnabled = enabled;
    }

    public void setIsAlign(boolean isAlign){
        this.isAlign = isAlign;
    }
    public boolean getIsAlign(){
        return isAlign;
    }

    /**
     * Calculates chassis speed of drivetrain using the current SwerveModuleStates
     * @return ChassisSpeeds object
     * This is often used as an input for other methods
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the state of each module
     * @return An array of 4 SwerveModuleStates
     */
    public SwerveModuleState[] getModuleStates(){
        return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    }

    public SwerveSetpoint getCurrSetpoint(){
        return currentSetpoint;
    }

    /**
     * @return the yaw of the robot, aka heading, the direction it is facing
     */
    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    /**
     * @return an array of modules
     */
    public Module[] getModules(){
        return modules;
    }

    /**
     * Resets the yaw of the robot.
     *
     * @param rotation the new yaw angle as Rotation2d
     */
    public void setYaw(Rotation2d rotation) {
        resetOdometry(new Pose2d(getPose().getTranslation(), rotation));
    }

    /**
     * Resets the odometry to the given pose.
     *
     * @param pose the pose to reset to.
     */
    public void resetOdometry(Pose2d pose) {
        // NOTE: must use pigeon yaw for odometer!
        currentHeading = pose.getRotation().getRadians();
        poseEstimator.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
        modulePoses.reset();
    }

    /**
     * @return the pose of the robot as estimated by the odometry
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the angle to align to
     * @param newAngle The new angle in radians, can be set to null
     */
    public void setAlignAngle(Double newAngle){
        alignAngle = newAngle;
    }

    /**
     * Returns whether or not the robot is at the input align angle
     * @return true if it within tolerance the align angle, false otherwise
     */
    public boolean atAlignAngle(){
        if(alignAngle == null){
            return false;
        }
        double diff = Math.abs(alignAngle - getYaw().getRadians());
        return diff < DriveConstants.HEADING_TOLERANCE || diff > 2*Math.PI - DriveConstants.HEADING_TOLERANCE;
    }

    /**
     * Gets the angle to align to
     * @return The angle in radians
     */
    public double getAlignAngle(){
        if(alignAngle != null){
            return alignAngle;
        }
        return 0;
    }

    /**
     * Sets vision to only use certain April tags
     * @param ids An array of the tags to only use
     */
    public void onlyUseTags(int[] ids){
        if(vision != null){
            vision.onlyUse(ids);
        }
    }
    /**
     * Returns if vision has seen an April tag in the last frame
     * @return true if vision saw a tag last frame or if vision is disabled
     */
    public boolean canSeeTag(){
        // if no vision system, then return true
        if (vision == null) return true;

        return vision.canSeeTag() || !visionEnabled || !VisionConstants.ENABLED;
    }

    /**
     * Gets the pose at a previous time
     * @param timestamp The timestamp of the pose to get
     * @return The pose, null if there are no poses yet, or the current pose if timestamp < 0
     */
    public Pose2d getPoseAt(double timestamp){
        if(timestamp < 0){
            return getPose();
        }
        Optional<Pose2d> pose = poseEstimator.sampleAt(timestamp);
        if(pose.isPresent()){
            return pose.get();
        }else{
            return null;
        }
    }

    /**
     * Uses pigeon and rotational input to return a rotation that accounts for drift
     * @return A rotation
     */
    public double headingControl(double rot, double xSpeed, double ySpeed){
        if((!EqualsUtil.epsilonEquals(getAngularRate(0), 0, 0.0004)&&EqualsUtil.epsilonEquals(Math.hypot(xSpeed, ySpeed),0,0.1))||!EqualsUtil.epsilonEquals(rot, 0, 0.0004)){
             drive_turning = true;
             currentHeading = getYaw().getRadians();
        }
        else{
            drive_turning = false;
        }
        if (!drive_turning){
            rotationController.setSetpoint(currentHeading);
            double output = rotationController.calculate(getYaw().getRadians());
            rot = Math.abs(output) > Math.abs(rot) ? output : rot;
        }
        return rot;
    }

    /**
     * Resets the swerve modules from the absolute encoders
     */
    public void resetModulesToAbsolute() {
        Arrays.stream(modules).forEach(Module::resetToAbsolute);
    }

    // getters for the PID Controllers
    public PIDController getXController() {
        return xController;
    }
    public PIDController getYController() {
        return yController;
    }
    public PIDController getRotationController() {
        return rotationController;
    }

    /**
     * Set the desired pose to drive to
     * This will enable driver assist to go to the pose
     * @param supplier The supplier for the desired pose, use ()->null to not use a desired pose
     */
    public void setDesiredPose(Supplier<Pose2d> supplier){
        desiredPoSupplier = supplier;
    }

    /**
     * Set the desired pose to drive to
     * This will enable driver assist to go to the pose
     * @param pose The Pose2d to drive to
     */
    public void setDesiredPose(Pose2d pose){
        setDesiredPose(()->pose);
    }

    /**
     * Gets the current desired pose, or null if there is no desired pose
     * @return The Pose2d if it exists, null otherwise
     */
    public Pose2d getDesiredPose(){
        return desiredPoSupplier.get();
    }

    public boolean atSetpoint(){
        Pose2d pose = getDesiredPose();
        return pose != null && getPose().getTranslation().getDistance(pose.getTranslation()) < 0.025;
    }

    public SwerveModulePose getSwerveModulePose(){
        return modulePoses;
    }

    public double getAcceleration() {
        double accelX = gyroInputs.accelerationX;
        double accelY = gyroInputs.accelerationY;

        double angularVelocity = getAngularRate(3);
        double angularAccel = (angularVelocity - previousAngularVelocity) / Constants.LOOP_TIME;
        previousAngularVelocity = angularVelocity;
        
        double pigeonOffsetX = 0.082677;
        double pigeonOffsetY = 0.030603444;

        double totalX = accelX + Math.pow(angularVelocity, 2) * pigeonOffsetX + angularAccel * pigeonOffsetY;
        double totalY = accelY + Math.pow(angularVelocity, 2) * pigeonOffsetY - angularAccel * pigeonOffsetX;

        return Math.hypot(totalX, totalY);
    }
   
    @AutoLogOutput(key = "Drivetrain/AccelerationFaults")
    public boolean accelerationOverMax() {
        return getAcceleration() > DriveConstants.MAX_LINEAR_ACCEL;
    }

    public void setCenterOfMass(double height){
        centerOfMassHeight = height;
    }

    public void alignWheels(){
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(1.465));
        setModuleStates(new SwerveModuleState[]{
            state, state, state, state
        }, false);
    }
}
