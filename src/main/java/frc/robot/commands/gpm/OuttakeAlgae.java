package frc.robot.commands.gpm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeAlgae extends SequentialCommandGroup {
    public OuttakeAlgae(Outtake outtake, Intake intake){
        BooleanSupplier intakeStow = ()-> intake.isAtSetpoint(IntakeConstants.STOW_SETPOINT);
        addCommands(
            new ConditionalCommand(
                new OuttakeAlgaeArm(outtake),
                new OuttakeAlgaeIntake(intake),
                intakeStow)
        );
    }
}
