package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveArm extends Command {
    private Arm arm;
    private double setpoint;

    public MoveArm(Arm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }
}
