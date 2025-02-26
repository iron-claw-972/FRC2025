package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.module.Module;
import frc.robot.subsystems.module.ModuleSim;
import frc.robot.util.DriverAssist;
import frc.robot.util.EqualsUtil;
import frc.robot.util.SwerveModulePose;
import frc.robot.util.Vision;
import frc.robot.util.SwerveStuff.SwerveSetpoint;
import frc.robot.util.SwerveStuff.SwerveSetpointGenerator;

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

    private final Pigeon2 pigeon;

    // PID Controllers for chassis movement
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;



    // If vision is enabled for drivetrain odometry updating
    // DO NOT CHANGE THIS HERE TO DISABLE VISION, change VisionConstants.ENABLED instead
    private boolean visionEnabled = true;

    // Disables vision for the first few seconds after deploying
    private Timer visionEnableTimer = new Timer();

    // If the robot should aim at the speaker
    private boolean isAlign = false;
    // Angle to align to, null for directly toward speaker
    private Double alignAngle = null;
    // used for drift control
    private double currentHeading = 0;
    // used for drift control
    private boolean drive_turning = false;

    private SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator();

    // The pose buffer, used to store and get previous poses
    private TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(2);

    // The pose supplier to drive to
    private Supplier<Pose2d> desiredPoSupplier = ()->null;

    private SwerveModulePose modulePoses;

    // The previous pose to reset to if the current pose gets too far off the field
    private Pose2d prevPose = new Pose2d();

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private boolean slipped = false;

    private BaseStatusSignal[] statusSignals = null;

    private double previousAngularVelocity = 0;

    private boolean controlsEnabled = false;

    private double centerOfMassHeight = 0;

    /**
     * Creates a new Swerve Style Drivetrain.
     */
    public Drivetrain(Vision vision) {
        this.vision = vision;

        modules = new Module[4];

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
        
        // The Pigeon is a gyroscope and implements WPILib's Gyro interface
        pigeon = new Pigeon2(IdConstants.PIGEON, DriveConstants.PIGEON_CAN);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        
        // Our pigeon is mounted with y forward, and z upward
        MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
        mountPoseConfigs.deserialize("");
        pigeon.getConfigurator().apply(new MountPoseConfigs().withMountPosePitch(0).withMountPoseRoll(0).withMountPoseYaw(90));

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        
        // initial Odometry Location
        pigeon.setYaw(DriveConstants.STARTING_HEADING.getDegrees());
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.KINEMATICS,
                Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()),
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

        modulePoses = new SwerveModulePose(this, DriveConstants.MODULE_LOCATIONS);

        // Store all status signals that we need to wait for
        for(int i = 0; i < modules.length; i++){
            BaseStatusSignal[] signals = modules[0].getStatusSignals();
            // Initialize array length and add pigeon if this is the first module
            if(i == 0){
                statusSignals = new BaseStatusSignal[4*signals.length+1];
                statusSignals[0] = pigeon.getYaw();
            }
            // Add all signals from this module
            for(int j = 0; j < signals.length; j++){
                statusSignals[i*signals.length+j+1] = signals[j];
            }
        }
        StatusSignal.setUpdateFrequencyForAll(250, statusSignals[0]);
        ParentDevice.optimizeBusUtilizationForAll(pigeon);
        // LogManager.logSupplier("Drivetrain/SpeedX", () -> getChassisSpeeds().vxMetersPerSecond, 100, LogLevel.INFO);
        // LogManager.logSupplier("Drivetrain/SpeedY", () -> getChassisSpeeds().vyMetersPerSecond, 100, LogLevel.INFO);
        // LogManager.logSupplier("Drivetrain/Speed", () -> Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond), 100, LogLevel.INFO);
        // LogManager.logSupplier("Drivetrain/SpeedRot", () -> getChassisSpeeds().omegaRadiansPerSecond, 100, LogLevel.INFO);
    
    //     LogManager.logSupplier("Drivetrain/Pose2d", () -> {
    //         Pose2d pose = getPose();
    //         return new Double[]{
    //             pose.getX(),
    //             pose.getY(),
    //             pose.getRotation().getRadians()
    //         };
    //     }, 15, LogLevel.COMP);
     }

    public void close() {
        // close the gyro
        pigeon.close();

        // close each of the modules
        for (int i = 0; i < modules.length; i++) {
            modules[i].close();
        }
    }

    @Override
    public void periodic() {
        //updateOdometryVision();
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
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        // Wait for all modules to update
        BaseStatusSignal.waitForAll(0.022, statusSignals);
        
        // Adding synchronized to a method does the same thing as synchronized(this), so this section won't run with other synchronized methods
        synchronized(this){
            // Updates pose based on encoders and gyro. NOTE: must use yaw directly from gyro!
            // Also stores the current pose in the buffer
            poseBuffer.addSample(Timer.getFPGATimestamp(), poseEstimator.update(Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()), updateModulePositions()));
        }
    }

    /**
     * Updates odometry using vision
     */
    public synchronized void updateOdometryVision() {
        // Start the timer if it hasn't started yet
        visionEnableTimer.start();

        // Update the swerve module poses
        modulePoses.update();

        if(modulePoses.slipped()){
            slipped = true;
        }

        Pose2d pose2 = getPose();

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

        if (Robot.isSimulation()) {
            pigeon.getSimState().addYaw(
                    +Units.radiansToDegrees(currentSetpoint.chassisSpeeds().omegaRadiansPerSecond * Constants.LOOP_TIME));
        }
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
     * Drives the robot
     * @param driver The driver config to get the controls from
     */
    public void drive(BaseDriverConfig driver){
        // Return and do nothing if controls are disabled
        if(!controlsEnabled) {
            return;
        }

        // If controls are enabled, run the controller code that was in DefaultDriveCommand in 2024

        double forwardTranslation = driver.getForwardTranslation();
        double sideTranslation = driver.getSideTranslation();
        double rotation = -driver.getRotation();

        double slowFactor = driver.getIsSlowMode() ? DriveConstants.SLOW_DRIVE_FACTOR : 1;

        forwardTranslation *= slowFactor;
        sideTranslation *= slowFactor;
        rotation *= driver.getIsSlowMode() ? DriveConstants.SLOW_ROT_FACTOR : 1;

        int allianceReversal = Robot.getAlliance() == Alliance.Red ? 1 : -1;
        forwardTranslation *= allianceReversal;
        sideTranslation *= allianceReversal;

        ChassisSpeeds driverInput = new ChassisSpeeds(forwardTranslation, sideTranslation, rotation);
        ChassisSpeeds corrected = DriverAssist.calculate(this, driverInput, getDesiredPose(), true);

        // If the driver is pressing the align button or a command set the drivetrain to align
        // Not currently used this year
        if (driver.getIsAlign() || getIsAlign()) {
            driveHeading(
                forwardTranslation,
                sideTranslation,
                getAlignAngle(),
                true);
        } else {
            drive(
                corrected.vxMetersPerSecond,
                corrected.vyMetersPerSecond,
                corrected.omegaRadiansPerSecond,
                true,
                false);
        }
    }

    public void enableDriveControls(boolean enable){
        controlsEnabled = enable;
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
        double speed = 0;
        switch(id){
            case 0:
                speed = pigeon.getAngularVelocityXWorld().getValueAsDouble();
                break;
            case 1:
                speed = pigeon.getAngularVelocityYWorld().getValueAsDouble();
                break;
            case 2:
                speed = pigeon.getAngularVelocityZWorld().getValueAsDouble();
                break;
        }
        // outputs in deg/s, so convert to rad/s
        return Units.degreesToRadians(speed);
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
    public synchronized void resetOdometry(Pose2d pose) {
        // NOTE: must use pigeon yaw for odometer!
        currentHeading = pose.getRotation().getRadians();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()), getModulePositions(), pose);
        modulePoses.reset();
    }

    /**
     * @return the pose of the robot as estimated by the odometry
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the angle to align to for the speaker
     * @param newAngle The new angle in radians, set to null to aim directly at the speaker
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
     * Gets the angle to align to for the speaker
     * @return The angle in radians
     */
    public double getAlignAngle(){
        if(alignAngle != null){
            return alignAngle;
        }
        return -Math.PI/2;
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
        Optional<Pose2d> pose = poseBuffer.getSample(timestamp);
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

    public Boolean atSetpoint(){
        return getDesiredPose()!=null && Math.abs(getDesiredPose().getX()-getPose().getX())<0.0127 && Math.abs(getDesiredPose().getY()-getPose().getY())<0.0127;

    }

    public SwerveModulePose getSwerveModulePose(){
        return modulePoses;
    }

    public double getAcceleration() {
        double accelX = pigeon.getAccelerationX().getValueAsDouble();
        double accelY = pigeon.getAccelerationY().getValueAsDouble();

        double angularVelocity = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        double angularAccel = (angularVelocity - previousAngularVelocity) / Constants.LOOP_TIME;
        previousAngularVelocity = angularVelocity;
        
        double pigeonOffsetX = 0.082677;
        double pigeonOffsetY = 0.030603444;

        double totalX = accelX + Math.pow(angularVelocity, 2) * pigeonOffsetX + angularAccel * pigeonOffsetY;
        double totalY = accelY + Math.pow(angularVelocity, 2) * pigeonOffsetY - angularAccel * pigeonOffsetX;

        return Math.hypot(totalX, totalY);
    }
   
   
   
   
    public boolean accelerationOverMax() {
        return getAcceleration() > DriveConstants.MAX_LINEAR_ACCEL;
    }

    public void setCenterOfMass(double height){
        centerOfMassHeight = height;
    }
}
