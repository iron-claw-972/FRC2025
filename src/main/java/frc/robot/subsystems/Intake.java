package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {

    public enum Mode {
        DISABLED(0),
        INTAKE(0.8),
        INTAKE_UP(0),
        PICKED_UP_CORAL(0.8),
        WAIT(0.8),
        PAUSE(0),
        REVERSE_MOTORS(-0.8);

        private final double power;

        Mode(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    private final TalonFX topMotor = new TalonFX(70);
    private final TalonFX botMotor = new TalonFX(71);
    private final TalonFX intakeStower = new TalonFX(68);

    private final PIDController stowPID = new PIDController(0.1, 0.0, 0.0); // Example gains; tune as needed
    private final Timer waitTimer = new Timer();

    private Mode mode;

    public Intake() {
        setMode(Mode.DISABLED);
        stowPID.setTolerance(0.5);

        if (RobotBase.isSimulation()) {
            // TODO: Add simulation-specific behavior if needed
        }

        waitTimer.start();

        publish();
    }

    private void publish() {
        // TODO: Add SmartDashboard or Shuffleboard publishing here if needed
    }

    public void setMode(Mode mode) {
        this.mode = mode;

        // Set motor powers based on the mode
        topMotor.set(ControlMode.PercentOutput, mode.getPower());
        botMotor.set(ControlMode.PercentOutput, -mode.getPower());

        if (mode == Mode.INTAKE_UP) {
            stowPID.reset();
            stowPID.setSetpoint(90); // Example angle; tune as needed
        } else {
            stowPID.reset();
            stowPID.setSetpoint(0);
        }

        waitTimer.reset();
    }

    @Override
    public void periodic() {
        publish();

        switch (mode) {
            case DISABLED:
                break;

            case INTAKE:
                if (hasCoral()) {
                    setMode(Mode.PICKED_UP_CORAL);
                }
                break;

            case PICKED_UP_CORAL:
                if (!hasCoral()) {
                    setMode(Mode.WAIT);
                } else if (waitTimer.hasElapsed(2)) {
                    setMode(Mode.PAUSE);
                }
                break;

            case PAUSE:
                if (waitTimer.hasElapsed(0.2)) {
                    setMode(Mode.REVERSE_MOTORS);
                }
                break;

            case REVERSE_MOTORS:
                if (!hasCoral()) {
                    setMode(Mode.WAIT);
                } else if (waitTimer.hasElapsed(5)) {
                    setMode(Mode.WAIT);
                }
                break;

            case WAIT:
                if (waitTimer.hasElapsed(0.1)) {
                    setMode(Mode.DISABLED);
                }
                break;

            case INTAKE_UP:
                intakeStower.set(stowPID.calculate(getStowPosition()));
                break;

            default:
                break;
        }
    }

    public boolean intakeInactive() {
        return (mode == Mode.DISABLED || mode == Mode.INTAKE_UP);

    public double getStowPosition() {
        // Example method to retrieve position in degrees; update as needed for your setup
        return Units.rotationsToDegrees(intakeStower.getSelectedSensorPosition());
    }

    public boolean hasCoral() {
        // TODO: Implement sensor logic to detect coral presence
        return false;
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation-specific periodic logic if needed
    }
}
