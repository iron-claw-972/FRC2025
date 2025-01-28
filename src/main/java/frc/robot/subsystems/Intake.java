package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {

    // TODO put in proper id
    private final TalonFX rollMotor = new TalonFX(70);
    private final TalonFX stowMotor = new TalonFX(68);
    private TalonFXSimState stowEncoderSim;

    private final PIDController stowPID = new PIDController(0, 0, 0);
    private final double motorVoltage = 12.0;

    public Intake() {

        // TODO set proper tolerance
        stowPID.setTolerance(.5);

        if (RobotBase.isSimulation()) {
            // TODO: Add simulation-specific behavior if needed
            stowEncoderSim = stowMotor.getSimState();
        }

        publish();
    }

    private void publish() {
        // TODO: Add SmartDashboard or Shuffleboard publishing here if needed
    }

    @Override
    public void periodic() {
        publish();
        stowMotor.set(stowPID.calculate(getStowPosition()));

    }

    /**
     * Gets the rotation of the intake
     * @return the rotation of the intake, in degrees
     */
    public double getStowPosition() {
        return Units.rotationsToDegrees(stowMotor.getPosition().getValueAsDouble());
    }


    public boolean hasCoral() {
        // TODO: Implement sensor logic to detect coral presence
        return false;
    }

    /**
     * Sets the desired angle of the intake, mostly to stow or unstow
     * @param angle desired angle of the intake
     */
    public void setAngle(double angle) {
        stowPID.setSetpoint(angle);
    }

    /**
     * Sets the speed of the roller motor
     * @param power The desired speed of the roller, between 1 and 0
     */
    public void setSpeed(double power) {
        rollMotor.set(power);
    }

    /**
     * Stows the intake and deactivates the rollers
     */
    public void stow() {
        stowPID.setSetpoint(90);
        deactivate();
    }

    /**
     * Unstows the intake, does not affect the rollers
     */
    public void unstow(){
        stowPID.setSetpoint(0);
    }

    /**
     * Deactivates the rollers only, does not stow or unstow
     */
    public void deactivate() {
        rollMotor.set(0);
    }

    /**
     * Ensures the intake is not stowed and activates the rollers
     */
    public void activate() {
        stowPID.setSetpoint(0);
        rollMotor.set(.8);
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation-specific periodic logic if needed
        
    }
}
