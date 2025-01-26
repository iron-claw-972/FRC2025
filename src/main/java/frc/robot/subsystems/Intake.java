package frc.robot.subsystems.gpm;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj.Timer;




public class Intake extends SubsystemBase {


    public enum Mode {
        DISABLED(0,0),
        INTAKE(.8,.3),
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


    /** Intake motor is a Vortex*/
    private final CANSparkFlex motor = new CANSparkFlex(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);


    // change the motor from neo550 to whatever it actually is
    private static final DCMotor dcMotor = DCMotor.getNeoVortex(1);


    /** Centering motor is a NEO */
    private final CANSparkMax centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private static final DCMotor dcMotorCentering = DCMotor.getNEO(1);
   
    /** beam break sensor detects whether a note is present */
    private final DigitalInput sensor  = new DigitalInput(IntakeConstants.SENSOR_ID);


    private final double MASS_SHAFT = 0.4; // in kilograms
    private final double LENGTH_SHAFT = Units.inchesToMeters(25.5);
    private final double MOI_SHAFT = (1.0 / 12.0) * MASS_SHAFT * LENGTH_SHAFT * LENGTH_SHAFT;
    private final double MOI_TOTAL = MOI_SHAFT * 4;


    private final double MASS_CENTERING_WHEELS = 0.1; // in kilograms
    private final double RADIUS_CENTERING_WHEELS = Units.inchesToMeters(2);
    private final double MOI_CENTERING_WHEEL = 0.5 * MASS_CENTERING_WHEELS * RADIUS_CENTERING_WHEELS
            * RADIUS_CENTERING_WHEELS;
    private final double MOI_CENTERING_TOTAL = MOI_CENTERING_WHEEL * 4;


    private final double motorVoltage = 12.0;


    private double motorRPMSim;
    private double centeringMotorRPMSim;


    private DIOSim intakeSensorDioSim;
    private FlywheelSim flywheelSim;
    private FlywheelSim centeringFlywheelSim;


    private Mode mode;


    private Timer waitTimer = new Timer();
    private Timer point2Timer = new Timer();


    public Intake() {
        // set the motor parameters
        // motor.setIdleMode(IntakeConstants.idleMode);***
        centeringMotor.setIdleMode(IntakeConstants.idleMode);


        // set the mode to Idle; this will turn off the motors
        setMode(Mode.DISABLED);


        // digital inputs
        // addChild("Intake motor", motor);
        // addchild("Centering motor", centeringMotor);
        addChild("Intake sensor", sensor);


        // Simulation objects
        if (RobotBase.isSimulation()) {
            intakeSensorDioSim = new DIOSim(sensor);
            intakeSensorDioSim.setValue(true);


            // assuming gearing is 1:1 for both
            flywheelSim = new FlywheelSim(dcMotor, 1.0, MOI_TOTAL);
            centeringFlywheelSim = new FlywheelSim(dcMotorCentering ,  1.0, MOI_CENTERING_TOTAL);
        }


        waitTimer.start();
        point2Timer.start();


        publish();
    }


    // publish sensor to Smart Dashboard
    private void publish() {
        SmartDashboard.putBoolean("Intake Sensor", sensor.get());
        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("Intake motor RPM", motorRPMSim);
            SmartDashboard.putNumber("Intake centering motor RPM", centeringMotorRPMSim);
        }
    }


    public void setMode(Mode mode) {
        this.mode = mode;


        // set the motor powers to be the value appropriate for this mode
        motor.set(mode.power);
        centeringMotor.set(mode.centeringPower);


        // restart the timer
        waitTimer.reset();
    }


    public boolean hasNote() {
        return !sensor.get();
    }


    @Override
    public void periodic() {
        publish();


        /* */
        switch (mode) {
            case DISABLED:
                // don't have to do anything
                break;


            case INTAKE:
                // motors are spinning and we are waiting to pick up a note
                if (hasNote()){
                    setMode(Mode.PickedUpNote);
                }
                break;


            case PickedUpNote:
                if (!hasNote()) {
                    setMode(Mode.Wait);
                } else if (waitTimer.hasElapsed(2)) {
                    setMode(Mode.Pause);
                }
                break;
           
            case Pause:
                if (waitTimer.hasElapsed(.2)) {
                    setMode(Mode.ReverseMotors);
                }
                break;


            case ReverseMotors:
                if (!hasNote()){
                    setMode(Mode.Wait);
                } else if (waitTimer.hasElapsed(5)) {
                    setMode(Mode.Wait);
                }
                break;


            case Wait:
                if (waitTimer.hasElapsed(0.1)) {
                    setMode(Mode.DISABLED);
                }
                break;


            default:
                break;
        }
    }


    /**
     * Get the intake motor current
     * @return motor current
     * @Deprecated This method is not used, and the simulation value is wrong
     */
    // @Deprecated
    // public double getCurrent() {
    //     if (RobotBase.isReal()) {
    //         return Math.abs(motor.getOutputCurrent());
    //     } else {
    //         return mode.power / motorVoltage;
    //     }
    // }***


    /**
     * Get the centering motor current
     * @return motor current
     * @Deprecated This method is not used, and the simulatin value is wrong.
     */
    public double getCenteringCurrent() {
        if (RobotBase.isReal()) {
            return Math.abs(centeringMotor.getOutputCurrent());
        } else {
            return mode.centeringPower / motorVoltage;
        }
    }


    public boolean intakeInactive() {
        return mode == Mode.DISABLED;
    }


    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(mode.power * motorVoltage);
        centeringFlywheelSim.setInputVoltage(mode.centeringPower * motorVoltage);


        flywheelSim.update(0.020);
        centeringFlywheelSim.update(0.020);


        motorRPMSim = flywheelSim.getAngularVelocityRPM();
        centeringMotorRPMSim = centeringFlywheelSim.getAngularVelocityRPM();


        if (mode == Mode.INTAKE) {
            if (point2Timer.hasElapsed(.2)) {
                intakeSensorDioSim.setValue(false);
                point2Timer.reset();
            }
        }
        else if (mode==Mode.DISABLED) {
            intakeSensorDioSim.setValue(true);
        }
    }


    public void close() {
        sensor.close();
    }
}