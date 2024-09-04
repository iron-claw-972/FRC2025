package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPose;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Aligns to the y coordinate of the closest April tag and turns toward it
 */
public class AlignToTag extends SequentialCommandGroup {
    private Drivetrain m_drive;

    /**
     * Aligns to the y coordinate of the closest April tag and turns toward it
     * @param drive The drivetrain
     */
    public AlignToTag(Drivetrain drive) {
        m_drive = drive;
        addCommands(new GoToPose(()->getPose(), drive).withTimeout(2));
    }

    /**
     * Gets the pose to align to,
     * with the x of the robot, y of the tag,
     * and rotation toward the tag
     * @return The Pose2d
     */
    public Pose2d getPose() {
        Pose2d closest = null;
        double dist = Double.POSITIVE_INFINITY;
        for(int i = 0; i < FieldConstants.APRIL_TAGS.size(); i++) {
            Pose2d tag = FieldConstants.APRIL_TAGS.get(i).pose.toPose2d();
            double d = tag.getTranslation().getDistance(m_drive.getPose().getTranslation());
            if(d < dist) {
                dist = d;
                closest = tag;
            }
        }
        double angle = Math.PI - closest.getRotation().getRadians();
        return new Pose2d(m_drive.getPose().getX(), closest.getY(), new Rotation2d(angle));
    }
}
