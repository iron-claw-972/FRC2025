package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Vision.DetectedObject;

public class AcquireGamePiece extends SequentialCommandGroup {
    /**
     * Intakes a game piece
     * 
     * @param gamePiece The supplier for the game piece to intake
     * @param drive The drivetrain
     * @param intake The intake
     * @param index The indexer
     * @param arm The arm
     */
    public AcquireGamePiece(Supplier<DetectedObject> gamePiece, Drivetrain drive, Intake intake, Indexer indexer, Elevator elevator, Outtake outtake, Arm arm){
        addCommands(new IntakeCoral(intake, indexer, elevator, outtake, arm).deadlineFor(new DriveToCoral(gamePiece, drive)));
    }
}