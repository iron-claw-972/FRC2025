package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {

    // TODO: put in proper id
    private final TalonFX rollMotor = new TalonFX(70);
    private final TalonFX stowMotor = new TalonFX(68);
    private TalonFXSimState stowEncoderSim;

    // TODO: set proper tolerance
    private final double positionTolerance = 0.5;

    private final PIDController stowPID = new PIDController(0, 0, 0);
    private final double motorVoltage = 12.0;

    public Intake() {
        if (RobotBase.isSimulation()) {
            // TODO: Add more simulation-specific behavior if needed
            stowEncoderSim = stowMotor.getSimState();
        }
        // IF USING SHUFFLEBOARD VVV
        // setupShuffleboard();
    }

    private void publish() {
        // TODO: Add SmartDashboard or Shuffleboard publishing here as needed
        SmartDashboard.putNumber("Stow Motor Position", getStowPosition());
        SmartDashboard.putNumber("Target Angle", stowPID.getSetpoint());
        SmartDashboard.putBoolean("Has Coral", hasCoral());
        SmartDashboard.putNumber("Roller Motor Power", rollMotor.get());
        SmartDashboard.putBoolean("Is Stowed", isAtSetpoint(90));
        SmartDashboard.putBoolean("Is Unstowed", isAtSetpoint(0));
        SmartDashboard.putBoolean("Is Roller Active", rollMotor.get() > 0);
    }

    // IF USING SHUFFLEBOARD VVV
    // private void setupShuffleboard() {
    // var intakeTab = Shuffleboard.getTab("Intake");

    // intakeTab.addNumber("Stow Motor Position", this::getStowPosition);
    // intakeTab.addNumber("Target Angle", stowPID::getSetpoint);
    // intakeTab.addBoolean("Has Coral", this::hasCoral);
    // intakeTab.addNumber("Roller Motor Power", rollMotor::get);
    // intakeTab.addBoolean("Is Stowed", () -> isAtSetpoint(90));
    // intakeTab.addBoolean("Is Unstowed", () -> isAtSetpoint(0));
    // intakeTab.addBoolean("Is Roller Active", () -> rollMotor.get() > 0);
    // }

    @Override
    public void periodic() {
        publish();
        stowMotor.set(stowPID.calculate(getStowPosition()));
    }

    /**
     * Gets the rotation of the intake.
     * 
     * @return the rotation of the intake (in degrees).
     */
    public double getStowPosition() {
        return Units.rotationsToDegrees(stowMotor.getPosition().getValueAsDouble());
    }

    /**
     * Checks if intake has coral.
     * 
     * @return Boolean (True if has Coral, False otherwise)
     */
    public boolean hasCoral() {
        // TODO: Implement sensor logic to detect coral presence
        return false;
    }

    /**
     * Checks if motor is at current setpoint.
     * 
     * @return Boolean (True if at setpoint, False otherwise)
     */
    private boolean isAtSetpoint(double setpoint) {
        return Math.abs(getStowPosition() - setpoint) < positionTolerance;
    }

    /**
     * Sets the desired angle of the intake, mostly to stow or unstow.
     * 
     * @param angle desired angle of the intake
     */
    public void setAngle(double angle) {
        stowPID.setSetpoint(angle);
    }

    /**
     * Sets the speed of the roller motor.
     * 
     * @param power The desired speed of the roller, between 0 and 1
     */
    public void setSpeed(double power) {
        rollMotor.set(power);
    }

    /**
     * Stows the intake and deactivates the rollers.
     */
    public void stow() {
        stowPID.setSetpoint(90);
        deactivate();
    }

    /**
     * Unstows the intake, does not affect the rollers.
     */
    public void unstow() {
        stowPID.setSetpoint(0);
    }

    /**
     * Deactivates the rollers only, does not stow or unstow.
     */
    public void deactivate() {
        rollMotor.set(0);
    }

    /**
     * Ensures the intake is not stowed and activates the rollers.
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
