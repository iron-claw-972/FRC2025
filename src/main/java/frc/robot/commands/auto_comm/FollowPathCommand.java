// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_comm;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SupplierCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;

/** Add your docs here. */
public class FollowPathCommand extends SequentialCommandGroup {
    Drivetrain drive;
    PathPlannerPath path;

    public FollowPathCommand(String name, Drivetrain drive){
        this(name, false, drive);
    }
    
    public FollowPathCommand(String pathName, boolean resetOdemetry, Drivetrain drive){
        this.drive = drive;
        this.path = PathGroupLoader.getPathGroup(pathName);
        addCommands(
            new InstantCommand(()->resetOdemetry(resetOdemetry)),
            new SupplierCommand(()->AutoBuilder.followPath(path), drive) // "problem" (254)
            // or pp's interaction with the drivetrain
            // or pp config
            );
    }

    public void resetOdemetry(boolean resetOdemetry){
        if (resetOdemetry){
            PathPoint point;
            if(RobotContainer.getAllianceColorBooleanSupplier().getAsBoolean()){
                point = path.getPoint(0).flip();
            }else{
                point = path.getPoint(0);
            }
            // TODO: Test if this resets it correctly
            drive.resetOdometry(new Pose2d(point.position, point.rotationTarget.rotation()));
        }
    }
    }
