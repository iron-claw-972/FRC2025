package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IdConstants;


public class OuttakeAlphaNew extends Outtake {

    private SparkFlex  motor = new SparkFlex(IdConstants.OUTTAKE_MOTOR_ALPHA, MotorType.kBrushless);
    private int ticks = 0;
    private enum State {LOADED, MOVING, REVERSING, DONE }
    private State state = State.DONE;


    /** Coral detected before the rollers */
    private DigitalInput digitalInputLoaded = new DigitalInput(IdConstants.OUTTAKE_DIO_LOADED);
    private DIOSim dioInputLoaded;

    /** Coral detected after the rollers */
    private DigitalInput digitalInputEjecting = new DigitalInput(IdConstants.OUTTAKE_DIO_EJECTING);
    private DIOSim dioInputEjecting;

    public OuttakeAlphaNew(){
        motor.configure(new SparkFlexConfig()
            .inverted(false)
            .idleMode(IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        if (RobotBase.isSimulation()){
            // object that will control the loaded sensor
            dioInputLoaded = new DIOSim(digitalInputLoaded);
            // object that will control the ejecting sensor
            dioInputEjecting = new DIOSim(digitalInputEjecting);
            // assume coral is loaded
            dioInputLoaded.setValue(false);
            // we are not ejecting
            dioInputEjecting.setValue(true);
        }
    }

    private double getMotorSpeed(){
        return motor.get();
    }

    @Override
    public void periodic(){
        ticks++;
        switch (state) {
            case LOADED:
            // waiting for ejected to become true
            if (coralEjecting()) {
                // at this point, ticks represents how long it took to move the coral to the ejecting sensor.
                SmartDashboard.putNumber("Coral Ejection Time", ticks * 0.020);
                // reset the timer
                ticks = 0;
                // we know the coral is moving
                state = State.MOVING;
            }
            break;
            case MOVING:
            // waiting for ejected to become false (success)
            if (!coralEjecting()){
                // coral has gone all the way through.
                SmartDashboard.putNumber("Coral Transit Time", ticks * 0.020);
                state = State.DONE;
            }
            // waiting for a timeout; 13 ticks is 0.26 seconds. It only takes 0.18 seconds to eject a coral.
            if (ticks > 13) {
                // reverse the motor, at -0.1: sometimes did not have the power to reverse, at -0.15: ejected all the way back, hit the funnel
                setMotor(0.125);


                SmartDashboard.putNumber("Coral Transit Time", ticks * 0.020);


                state = State.REVERSING;
            }
            break;

            case REVERSING:
            // waiting for ejected to be false
            if (!coralEjecting()){
                // we are done. When command finishes, the motor will be stopped.
                state = State.DONE;
            }
            break;

            case DONE:
            break;
        }
        SmartDashboard.putBoolean("Coral loaded", coralLoaded());
        SmartDashboard.putBoolean("Coral ejected", coralEjecting());

    }

    @Override
    public void simulationPeriodic() {
        
        if (getMotorSpeed() > 0.05) {
            if (ticks > 250) {
                ticks = 0;
            }
            // motor is outtaking
            // motor is spinning, ejecting will be true. after 0.14 seconds
            if (ticks == 7) {
                dioInputEjecting.setValue(false);
            }
            if (ticks == 14){
                // after 0.14 second
                dioInputLoaded.setValue(true);
            }
            if (ticks == 16){
                // after 0.18 seconds
                dioInputEjecting.setValue(true);
            }   
        }
    }
            
    /** Set the motor power to move the coral */
    public void setMotor(double power){
        motor.set(power);
    }

    /** start spinning the rollers to eject the coral */
    public void outtake(){
        ticks = 0;
        if (coralLoaded()) {
            // coral is present, ejecting makes sense
            state = State.LOADED;
            // wheels start spinning
            setMotor(-0.2);
        }
        else {
            // no coral present, ejecting does not make sense
            state = State.DONE;
        }
        // assumes the coral is present
        // if the coral is not present, we should not bother to spin the rollers
        // this starts the motor... what needs to be done later?

    }

    public boolean coralLoaded(){
       return !digitalInputLoaded.get();
    }

    /**
     *  Coral is at the ejecting beam break sensor.
     * @return coral is interrupting the beam breaker.
     */
    public boolean coralEjecting() {
        return !digitalInputEjecting.get();
    }

    public void fakeLoad() {
        if (RobotBase.isSimulation()) {
            dioInputLoaded.setValue(false);
        }
    }

    public boolean isFinished() {
        return state == State.DONE;
    }

    public void close() {
        motor.close();
        digitalInputLoaded.close();
        digitalInputEjecting.close();
    }
}