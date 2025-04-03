package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final TalonFX rollerMotor = new TalonFX(IdConstants.INTAKE_ROLLER);
    private final TalonFX pivotMotor = new TalonFX(IdConstants.INTAKE_PIVOT, Constants.CANIVORE_CAN);
    
    private SingleJointedArmSim stowArmSim;
    private Mechanism2d stowMechanism2d;
    private MechanismLigament2d stowWheelLigament;

    private final double positionTolerance = 5;

    private final PIDController stowPID = new PIDController(0.015, 0, 0);
    private double power;

    private LaserCan laserCan;
    private boolean hasCoral = false;
    private boolean isMoving = false;
    private Timer laserCanSimTimer;
    private DCMotor dcMotor = DCMotor.getKrakenX60(1);
    private ArmFeedforward feedforward = new ArmFeedforward(0,
            Constants.GRAVITY_ACCELERATION * IntakeConstants.CENTER_OF_MASS_DIST * IntakeConstants.MASS
                    / IntakeConstants.PIVOT_GEAR_RATIO * dcMotor.rOhms / dcMotor.KtNMPerAmp / Constants.ROBOT_VOLTAGE,
            0);
    private double startPosition = 90;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake() {
        if (RobotBase.isSimulation()) {
            stowMechanism2d = new Mechanism2d(10, 10);
            stowWheelLigament = stowMechanism2d.getRoot("Root", 5, 5)
                    .append(new MechanismLigament2d("Intake", 4, startPosition));
            SmartDashboard.putData("Intake pivot", stowMechanism2d);
            stowArmSim = new SingleJointedArmSim(
                    dcMotor,
                    IntakeConstants.PIVOT_GEAR_RATIO,
                    IntakeConstants.MOMENT_OFiNERTIA,
                    IntakeConstants.ARM_LENGTH,
                    Math.toRadians(0),
                    Math.toRadians(90),
                    true,
                    Units.degreesToRadians(startPosition));
            laserCanSimTimer = new Timer();
        } else {
            // laserCan = new LaserCan(IdConstants.INTAKE_LASER_CAN);
            // try {
            //     laserCan.setRangingMode(RangingMode.SHORT);
            //     laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
            //     laserCan.setRegionOfInterest(new RegionOfInterest(-4, -4, 8, 8));
            // } catch (ConfigurationFailedException e) {
            //     DriverStation.reportError("Intake LaserCan configuration error", true);
            // }
        }
        rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.CounterClockwise_Positive));
        pivotMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        pivotMotor.setPosition(Units.degreesToRotations(startPosition) * IntakeConstants.PIVOT_GEAR_RATIO);
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
        stowPID.setTolerance(positionTolerance);
        
        setAngle(startPosition);

       rollerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));
    }

    /**
     * publishes stuff to smartdashboard
     */
    @SuppressWarnings("unused")
    private void publish() {
        SmartDashboard.putNumber("Stow Motor Position", getPivotAngle());
        SmartDashboard.putNumber("Target Angle", stowPID.getSetpoint());
        SmartDashboard.putNumber("Roller Motor Power", power);

        SmartDashboard.putBoolean("Has Coral", hasCoral());
        SmartDashboard.putBoolean("Stow Arm Sim - Is Stowed", isAtSetpoint(90));
        SmartDashboard.putBoolean("Stow Arm Sim - Is Unstowed", isAtSetpoint(0));
        SmartDashboard.putBoolean("Is Roller Active", rollerMotor.get() > 0);
    }

    @Override
    public void periodic() {
        //publish();
        // SmartDashboard.putNumber("angle", getPivotAngle());
        // SmartDashboard.putBoolean("Intake has coral", hasCoral());

        double position = getPivotAngle();
        power = stowPID.calculate(position) + feedforward.calculate(Units.degreesToRadians(position), 0);
        power = MathUtil.clamp(power, -1, 1);
        pivotMotor.set(power);
        if (laserCan != null) {
            Measurement measurement = laserCan.getMeasurement();
            hasCoral = measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
                    && measurement.distance_mm <= 1000 * IntakeConstants.DETECT_CORAL_DIST;
        }
        if(RobotBase.isSimulation()) {
            inputs.measuredPivotPosition = Units.radiansToDegrees(stowArmSim.getAngleRads());
        } else {
            inputs.measuredPivotPosition = Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()) / IntakeConstants.PIVOT_GEAR_RATIO;
        }
        inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/PivotSetpoint", stowPID.getSetpoint());        
    }

    @Override
    public void simulationPeriodic() {
        stowArmSim.setInputVoltage(power * Constants.ROBOT_VOLTAGE);
        stowArmSim.update(Constants.LOOP_TIME);
        stowWheelLigament.setAngle(Units.radiansToDegrees(stowArmSim.getAngleRads()));
        if (!isMoving) {
            laserCanSimTimer.reset();
            laserCanSimTimer.start();
            hasCoral = false;
        } else {
            if (isAtSetpoint()) {
                laserCanSimTimer.start();
            }
            hasCoral = laserCanSimTimer.hasElapsed(0.5) && !laserCanSimTimer.hasElapsed(1);
        }
    }

    /**
     * Gets the rotation of the intake.
     * 
     * @return the rotation of the intake (in degrees).
     */
    public double getPivotAngle() {
        return inputs.measuredPivotPosition;
    }

    public PIDController getPID() {
        return stowPID;
    }

    /**
     * Checks if intake has coral.
     * 
     * @return Boolean (True if has Coral, False otherwise)
     */
    @AutoLogOutput(key = "Intake/HasCoral")
    public boolean hasCoral() {
        return hasCoral;
    }

    /**
     * Checks if motor is at current setpoint.
     * 
     * @return Boolean (True if at setpoint, False otherwise)
     */
    public boolean isAtSetpoint(double setpoint) {
        return Math.abs(getPivotAngle() - setpoint) < positionTolerance;
    }

    /**
     * Returns whether or not the intake is at its setpoint
     * 
     * @return True if it is at the setpoint, false otherwise
     */
    @AutoLogOutput(key = "Intake/AtSetPoint")
    public boolean isAtSetpoint() {
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
        isMoving = Math.abs(power) < 0.01;

    }

    /**
     * Moves the intake up, doesn't stop it.
     */
    public void stow() {
        stowPID.setSetpoint(IntakeConstants.STOW_SETPOINT);
    }

    /**
     * Moves the intake down.
     */
    public void unstow() {
        stowPID.setSetpoint(IntakeConstants.INTAKE_SETPOINT);
    }

    /**
     * Stops the motor.
     */
    public void deactivate() {
        rollerMotor.set(0);
    }

    /**
     * Starts the motor.
     */
    public void activate(){
        rollerMotor.set(IntakeConstants.INTAKE_MOTOR_POWER);
    }
}

