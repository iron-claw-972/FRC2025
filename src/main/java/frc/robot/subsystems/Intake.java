package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;




public class Intake extends SubsystemBase {


    public enum Mode {
        DISABLED(0),
        INTAKE(.8),
        PickedUpCoral(.8),
        Wait(.8),
        Pause (0),
        ReverseMotors(-.8);


        private double power;


        Mode(double power) {
            this.power = power;
        }


        public double getPower() {
            return power;
        }

    }



    // TODO put in proper id
    private final TalonFX topMotor = new TalonFX(70);
    private final TalonFX botMotor = new TalonFX(71);


    private final double motorVoltage = 12.0;

    private Mode mode;


    public Intake() {

        // set the mode to Idle; this will turn off the motors
        setMode(Mode.DISABLED);


        // Simulation objects
        if (RobotBase.isSimulation()) {
            //TODO add sim stuff
        }



        publish();
    }


    // publish sensor to Smart Dashboard
    private void publish() {
        //TODO publish stuff
    }


    public void setMode(Mode mode) {
        this.mode = mode;


        // set the motor powers to be the value appropriate for this mode
        topMotor.set(mode.power);
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