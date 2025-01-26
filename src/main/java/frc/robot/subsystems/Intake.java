package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;





public class Intake extends SubsystemBase {


    public enum Mode {
        DISABLED(0),
        INTAKE(.8),
        INTAKE_UP(0),
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
    private final TalonFX intakeStower = new TalonFX(68);

    private final PIDController stowPID = new PIDController(0, 0, 0);

    private Timer waitTimer = new Timer();

    private final double motorVoltage = 12.0;

    private Mode mode;


    public Intake() {

        // set the mode to Idle; this will turn off the motors
        setMode(Mode.DISABLED);
        stowPID.setTolerance(.5);


        // Simulation objects
        if (RobotBase.isSimulation()) {
            //TODO add sim stuff
        }

        waitTimer.start();

        if (Constants.LOG_LEVEL.getValue() > 0) {
            //TODO fix logging
            /*
            LogManager.add("Intake/motorVolts", () -> motor.get() * Constants.ROBOT_VOLTAGE);
            LogManager.add("Intake/centeringMotorVolts", () -> centeringMotor.get() * Constants.ROBOT_VOLTAGE);
            
            LogManager.add("Intake/motorRPM", () -> motor.getAbsoluteEncoder().getVelocity(), Duration.ofSeconds(1));
            LogManager.add("Intake/centeringMotorRPM", () -> centeringMotor.getAbsoluteEncoder().getVelocity(), Duration.ofSeconds(1)); */
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
        botMotor.set(-mode.power);

        if (mode == Mode.INTAKE_UP){
            stowPID.reset();
            stowPID.setSetpoint(90);
        }else{
            stowPID.reset();
            stowPID.setSetpoint(0);
        }

        waitTimer.reset();
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
                if (hasCoral()){
                    setMode(Mode.PickedUpCoral);
                }
                break;


            case PickedUpCoral:
                if (!hasCoral()) {
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
                if (!hasCoral()){
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
            
            case INTAKE_UP:
                {
                    intakeStower.set(stowPID.calculate(getStowPosition()));
                }
                break;

            default:
                break;
        }
    }

    public double getStowPosition(){
        return Units.rotationsToDegrees(intakeStower.getPosition().getValueAsDouble());
    }

    public boolean intakeInactive() {
        return (mode == Mode.DISABLED || mode == Mode.INTAKE_UP);
    }

    public boolean hasCoral() {
        //TODO check if intake has a coral
        return false;
    }


    @Override
    public void simulationPeriodic() {
        //TODO add sim stuff
    }


}