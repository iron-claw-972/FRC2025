package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import lib.CTREModuleState;


/**
 * Swerve module for drivetrain to be used inside of simulation.
 * TODO: improve this simulation to be more realistic
 */
public class ModuleSim extends Module {

    private double currentSteerPositionRad = 0;
    private double currentDrivePositionMeters = 0;
    private double currentSpeed = 0;

    private SwerveModuleState desiredState;


    protected boolean stateDeadband = true;

    public ModuleSim(ModuleConstants ignored) {
        super(ignored);
    }

    /**
     * Updates the simulation
     */
    @Override
    public void periodic() {
        currentDrivePositionMeters += currentSpeed * Constants.LOOP_TIME;
        super.periodic();
    }

    @Override
    public void updateInputs(){
        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = currentDrivePositionMeters / DriveConstants.WHEEL_RADIUS;
        inputs.driveVelocityRadPerSec = currentSpeed / DriveConstants.WHEEL_RADIUS;
        inputs.driveAppliedVolts = currentSpeed / DriveConstants.MAX_SPEED * Constants.ROBOT_VOLTAGE;
        inputs.driveCurrentAmps = 0; // This simulation currently isn't good enough to calculate this
    
        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(currentSteerPositionRad);
        inputs.turnPosition = new Rotation2d(currentSteerPositionRad);
        inputs.turnVelocityRadPerSec = 0; // Simulated modules currently teleport
        inputs.turnAppliedVolts = 0;
        inputs.turnCurrentAmps = 0;

        // Update odometry inputs
        // Simulate as only getting one value per frame
        inputs.odometryTimestamps =
            new double[]{Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad =
            new double[]{inputs.drivePositionRad*DriveConstants.DRIVE_GEAR_RATIO};
        inputs.odometryTurnPositions =
            new Rotation2d[]{inputs.turnPosition};
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     * @param isOpenLoop   whether to use closed/open loop control for drive velocity
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(!DriveConstants.DISABLE_DEADBAND_AND_OPTIMIZATION){
            // If the module isn't moving, don't rotate it
            if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
                currentSpeed = 0;
                return;
            }
            // Optimize the reference state to avoid spinning further than 90 degrees
            desiredState = CTREModuleState.optimize(desiredState, new Rotation2d(currentSteerPositionRad));
        }

        currentSpeed = desiredState.speedMetersPerSecond;
        currentSteerPositionRad = desiredState.angle.getRadians();
    }

    public void resetToAbsolute() {
        // does nothing when robot does not have a swerve drivetrain
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
      }

    public double getDesiredVelocity() {
        return getDesiredState().speedMetersPerSecond;
      }
    
      public Rotation2d getDesiredAngle() {
        return getDesiredState().angle;
      }

    /**
     * Sets current speed to zero
     */
    public void stop() {
        currentSpeed = 0;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                currentSpeed,
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                currentDrivePositionMeters,
                new Rotation2d(currentSteerPositionRad)
        );
    }

    /**
     * Gets the simulated angle of the module.
     */
    public Rotation2d getAngle() {
        return new Rotation2d(currentSteerPositionRad);
    }

     /**
     * Sets state deadband
     */
    public void setStateDeadband(boolean enabled) {
        stateDeadband = enabled;
    }

    public TalonFX getDriveMotor(){
        return null;
    }

    public double getDriveVoltage(){
        return 0;
    }

    public double getDriveStatorCurrent(){
        return 0;
    }
    
    public double getSteerVelocity() {
        return 0;
    }
    public double getDriveVelocity() {
        return 0;
    }

}