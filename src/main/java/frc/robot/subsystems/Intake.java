package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // TODO: Use that sim object to update the sim state using .addRotorPosition()
    // with the sim velocity * time or .setRawRotorPosition() with the position.

    // TODO: Tune the PID

    // TODO: Add the sensor once they figure out what type it will be

    // TODO: Maybe simulate that sensor and make it activate 1 second after the
    // intake starts (low priority)

    // TODO: put in proper id

    private final TalonFX rollerMotor = new TalonFX(70);
    private final TalonFX stowMotor = new TalonFX(68);
    private final SingleJointedArmSim stowArmSim;
    private TalonFXSimState stowEncoderSim;

    // TODO: set proper tolerance
    private final double positionTolerance = 0.5;

    private final PIDController stowPID = new PIDController(0, 0, 0);
    private final double motorVoltage = 12.0;

    public Intake() {
        if (RobotBase.isSimulation()) {
            // TODO: Add more simulation-specific behavior if needed
            double velocity = 0; // replace with actual value
            double timeDelta = 0; // replace with actual value

            // TODO: Simulate arm's position using velocity & timeDelta provided above

            stowEncoderSim = stowMotor.getSimState();
        }
        // IF USING SHUFFLEBOARD VVV
        // setupShuffleboard();

        stowArmSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                1.0,
                calculateMomentOfInertia(),
                getStowMotorLength(),
                Math.toRadians(0),
                Math.toRadians(90),
                true,
                Math.PI/2);
    }

    /**
     * publishes stuff to smartdashboard
     */
    private void publish() {
        // TODO: Add SmartDashboard or Shuffleboard publishing here as needed
        SmartDashboard.putNumber("Stow Motor Position", getStowPosition());
        SmartDashboard.putNumber("Target Angle", stowPID.getSetpoint());
        SmartDashboard.putBoolean("Has Coral", hasCoral());
        SmartDashboard.putNumber("Roller Motor Power", rollerMotor.get());
        SmartDashboard.putBoolean("Is Stowed", isAtSetpoint(90));
        SmartDashboard.putBoolean("Is Unstowed", isAtSetpoint(0));
        SmartDashboard.putBoolean("Is Roller Active", rollerMotor.get() > 0);
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
     * Calculates intertia for the arm.
     * 
     * @return moment of inertia of the arm in kg·m²
     */
    private double calculateMomentOfInertia() {
        //TODO add propery numby
        double armMass = 5.0; // replace with actual value
        double armLength = 0.5; // replace with actual value
        return (1.0 / 3.0) * armMass * Math.pow(armLength, 2);
    }

    /**
     * Retrieves the length of the arm.
     * 
     * @return The length of the arm in meters.
     */
    private double getStowMotorLength() {
        return 0.5; // replace with actual value
    }

    /**
     * Checks if intake has coral.
     * 
     * @return Boolean (True if has Coral, False otherwise)
     */
    public boolean hasCoral() {
        // TODO: Implement logic base on the sensor once identified
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
     * @param power The desired speed of the roller, between 0 and 1.
     */
    public void setSpeed(double power) {
        rollerMotor.set(power);
    }

    /**sssssss
     * Moves the intake up and stops it.
     */
    public void stow() {
        stowPID.setSetpoint(0);
        deactivate();
    }

    /**
     * Moves the intake down.
     */
    public void unstow() {
        stowPID.setSetpoint(0);
    }

    /**
     * Stops the motor.
     */
    public void deactivate() {
        rollerMotor.set(0);
    }

    /**
     * Lowers the intake and starts the motor.
     */
    public void activate() {
        stowPID.setSetpoint(90);
        rollerMotor.set(.8);
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation-specific periodic logic if needed
        
    }
}
