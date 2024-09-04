package frc.robot.subsystems.gpm;

import java.time.Duration;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.LogManager;
import edu.wpi.first.wpilibj.Timer;


public class Intake extends SubsystemBase {

    public enum Mode {
        DISABLED(0,0),
        INTAKE(.6,.5),
        PickedUpNote(.8,.3),
        Wait(.8,.3),
        Pause (0,0),
        ReverseMotors(-.8,-.3);

        private double power;
        private double centeringPower;

        Mode(double power, double centeringPower) {
            this.power = power;
            this.centeringPower = centeringPower;
        }

        public double getPower() {
            return power;
        }

        public double getCenteringPower() {
            return centeringPower;
        }
    }

    // Intake motor is a Vortex
    private final CANSparkFlex motor = new CANSparkFlex(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    // Change the motor from neo550 to whatever it actually is
    private static final DCMotor dcMotor = DCMotor.getNeoVortex(1);

    // Centering motor is a NEO 
    private final CANSparkMax centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private static final DCMotor dcMotorCentering = DCMotor.getNEO(1);
    
    // Beam break sensor detects whether a note is present 
    private final DigitalInput sensor  = new DigitalInput(IntakeConstants.SENSOR_ID);

    private double motorRPMSim;
    private double centeringMotorRPMSim;

    private int countSim = 0;
    private DIOSim IntakeSensorDioSim;
    private boolean simDIOValue = true;
    private FlywheelSim flywheelSim;
    private FlywheelSim centeringFlywheelSim;

    private Mode mode;

    private Timer waitTimer = new Timer();

    public Intake() {
        centeringMotor.setIdleMode(IntakeConstants.idleMode);
        centeringMotor.setInverted(true);
        motor.setInverted(true);

        // set the mode to Idle; this will turn off the motors
        setMode(Mode.DISABLED);

        // digital inputs
        // addChild("Intake motor", motor);
        // addchild("Centering motor", centeringMotor);
        addChild("Intake sensor", sensor);

        // Simulation objects
        if (RobotBase.isSimulation()) {
            IntakeSensorDioSim = new DIOSim(sensor);
            
            // assuming gearing is 1:1 for both
            flywheelSim = new FlywheelSim(dcMotor, 1.0, IntakeConstants.MOI_TOTAL);
            centeringFlywheelSim = new FlywheelSim(dcMotorCentering ,  1.0, IntakeConstants.MOI_CENTERING_TOTAL);
        }

        waitTimer.start();

        if (Constants.DO_LOGGING) {
            LogManager.add("Intake/motorVolts", () -> motor.get() * Constants.ROBOT_VOLTAGE);
            LogManager.add("Intake/centeringMotorVolts", () -> centeringMotor.get() * Constants.ROBOT_VOLTAGE);
            
            LogManager.add("Intake/motorRPM", () -> motor.getAbsoluteEncoder().getVelocity(), Duration.ofSeconds(1));
            LogManager.add("Intake/centeringMotorRPM", () -> centeringMotor.getAbsoluteEncoder().getVelocity(), Duration.ofSeconds(1));            
        }

    }

    // Publish sensor to Smart Dashboard
    private void publish() {
        SmartDashboard.putBoolean("Intake Sensor", sensor.get());

        if (RobotBase.isSimulation()) {
            // SmartDashboard.putNumber("Intake motor RPM", motorRPMSim);
            // SmartDashboard.putNumber("Intake centering motor RPM", centeringMotorRPMSim);
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;

        motor.set(mode.power);
        centeringMotor.set(mode.centeringPower);

        waitTimer.reset();
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
         publish();
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(mode.power * IntakeConstants.motorVoltage);
        centeringFlywheelSim.setInputVoltage(mode.centeringPower * IntakeConstants.motorVoltage);

        flywheelSim.update(0.020);
        centeringFlywheelSim.update(0.020);

        motorRPMSim = flywheelSim.getAngularVelocityRPM();
        centeringMotorRPMSim = centeringFlywheelSim.getAngularVelocityRPM();

        // Changes sensor values every 1/2 second
        if (countSim++ > 25) {
            countSim = 0;

            IntakeSensorDioSim.setValue(simDIOValue);

            simDIOValue = !simDIOValue;
        }

    }

    public void close() {
        sensor.close();
    }
}
