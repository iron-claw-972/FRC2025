package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;





public class Intake extends SubsystemBase {

    // TODO put in proper id
    private final TalonFX topMotor = new TalonFX(70);
    private final TalonFX botMotor = new TalonFX(71);
    private final TalonFX stowMotor = new TalonFX(68);

    private final PIDController stowPID = new PIDController(0, 0, 0);

    private final double motorVoltage = 12.0;

    public Intake() {

        //TODO set proper tolerance
        stowPID.setTolerance(.5);

        if (RobotBase.isSimulation()) {
            // TODO: Add simulation-specific behavior if needed
        }


        publish();
    }

    private void publish() {
        // TODO: Add SmartDashboard or Shuffleboard publishing here if needed
    }

    @Override
    public void periodic() {
        publish();
        stowMotor.set(stowPID.calculate(getStowPosition()));


    }

    public double getStowPosition(){
        return Units.rotationsToDegrees(stowMotor.getPosition().getValueAsDouble());
    }

    public boolean hasCoral() {
        // TODO: Implement sensor logic to detect coral presence
        return false;
    }

    public void setAngle(double angle){
        stowPID.setSetpoint(angle);
    }

    public void setSpeed(double power){
        topMotor.set(power);
        botMotor.set(-power);
    }

    public void stow(){
        stowPID.setSetpoint(90);
    }

    public void deactivate(){
        topMotor.set(0);
        botMotor.set(0);
    }

    public void activate(){
        stowPID.setSetpoint(0);
        topMotor.set(.8);
        botMotor.set(-.8);
    }

    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation-specific periodic logic if needed
    }
}
