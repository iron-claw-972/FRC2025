package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;

public class StationIntake extends Command {
    private Outtake outtake;

    public StationIntake(Outtake outtake) {
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
        outtake.setMotor(0.7);
    }

    @Override
    public boolean isFinished() {
        return outtake.coralLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setMotor(0.02);
    }


}
