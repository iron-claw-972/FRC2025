package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

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
    private SingleJointedArmSim stowArmSim;
    private Mechanism2d stowMechanism2d;
    private MechanismLigament2d stowWheelLigament;
    private PIDController stowPIDController;

    private final double positionTolerance = 1;
    // TODO: replace these with actual values
    private final double gearRatio = 10;
    private final double momentOfInertia = 0.3;
    private final double armLength = 0.3;

    // TODO: tune
    private final PIDController stowPID = new PIDController(0.01, 0, 0.001);
    private double power;

    public Intake() {
        stowMotor.setPosition(0.25*gearRatio);
        if (RobotBase.isSimulation()) {
            stowMechanism2d = new Mechanism2d(10, 10);
            stowWheelLigament = stowMechanism2d.getRoot("Root", 5, 5).append(new MechanismLigament2d("Intake", 4, 90));
            SmartDashboard.putData("Intake pivot", stowMechanism2d);
            stowArmSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                gearRatio,
                momentOfInertia,
                armLength,
                Math.toRadians(0),
                Math.toRadians(90),
                true,
                Math.PI/2);
            }
            stowPID.setSetpoint(45);
        }

    /**
     * publishes stuff to smartdashboard
     */
    private void publish() {
        SmartDashboard.putNumber("Stow Motor Position", getStowPosition());
        SmartDashboard.putNumber("Target Angle", stowPID.getSetpoint());
        SmartDashboard.putNumber("Roller Motor Power", rollerMotor.get());

        SmartDashboard.putBoolean("Has Coral", hasCoral());
        SmartDashboard.putBoolean("Stow Arm Sim - Is Stowed", isAtSetpoint(90));
        SmartDashboard.putBoolean("Stow Arm Sim - Is Unstowed", isAtSetpoint(0));
        SmartDashboard.putBoolean("Is Roller Active", rollerMotor.get() > 0);
    }

    @Override
    public void periodic() {
        publish();
        power = stowPID.calculate(getStowPosition());
        stowMotor.set(power);
    }

    @Override
    public void simulationPeriodic(){
        stowArmSim.setInputVoltage(power*Constants.ROBOT_VOLTAGE);
        stowArmSim.update(Constants.LOOP_TIME);
        stowWheelLigament.setAngle(Units.radiansToDegrees(stowArmSim.getAngleRads()));
    }

    /**
     * Gets the rotation of the intake.
     * 
     * @return the rotation of the intake (in degrees).
     */
    public double getStowPosition() {
        // TalonFXSimState doens't work after the update, so this is now the best way to get the position in sim
        if(RobotBase.isReal()){
            return Units.rotationsToDegrees(stowMotor.getPosition().getValueAsDouble())/gearRatio;
        }else{
            return Units.radiansToDegrees(stowArmSim.getAngleRads());
        }
    }

    public PIDController getPID() {
        return stowPIDController;
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
     * Returns whether or not the intake is at its setpoint
     * @return True if it is at the setpoint, false otherwise
     */
    public boolean isAtSetpoint(){
        return stowPID.atSetpoint();
    }

    /**
     * Sets the desired angle of the intake, mostly to stow or unstow.
     * 
     * @param angle desired angle of the intake in degrees
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

    /**
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
}
