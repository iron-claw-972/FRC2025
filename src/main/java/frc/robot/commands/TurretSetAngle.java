package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Turret;

public class TurretSetAngle extends Command {
    private final Turret turret;
    private final double angle;

    public TurretSetAngle(Turret turret, double angle) {
        this.turret = turret;
        this.angle = angle;
        addRequirements(this.turret);
    }

    public void initialize() {
        turret.setAngle(angle);
      }
    
    public boolean isFinished() {
        return turret.atSetpoint();
    }
}
