package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Shooter;

public class SetShooterSpeed extends Command{

    private final Shooter m_shooter;
    private final double targetSpeed;

	/**
	* Spins the shooter up to the target speed, and waits before exiting.
	*
	* @param shooter      the shooter to spin up
	* @param targetSpeed  the target speed (in m/s) to spin up to.
	* @see                frc.robot.commands.gpm.PrepareShooter
	*/
    public SetShooterSpeed(Shooter shooter, double targetSpeed) {

        m_shooter = shooter;
        this.targetSpeed = targetSpeed;

    }

    @Override
    public void initialize(){
        m_shooter.setTargetRPM(targetSpeed);

    }

    @Override
    public void execute(){

    }

      @Override
    public boolean isFinished(){
        return m_shooter.atSetpoint();
    }

    @Override
    public void end(boolean interupted){
        if (interupted){
            m_shooter.setTargetRPM(0);
        }
    }
}
