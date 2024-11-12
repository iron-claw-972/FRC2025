// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.module.Module;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;



/** Add your docs here. */
public class SwerveTab extends ShuffleBoardTabs {
    
    private Drivetrain drive;

    private Module[] modules;

    private GenericEntry xOdemetry;
    private GenericEntry yOdemetry;
    private GenericEntry rotOdemetry;
    private GenericEntry[] steerSpeed = new GenericEntry[4];
    private GenericEntry[] driveSpeed = new GenericEntry[4];
    private GenericEntry[] driveAccel = new GenericEntry[4];
    private GenericEntry[] rotationalPosition = new GenericEntry[4];
    private GenericEntry[] voltage = new GenericEntry[4];
    private GenericEntry[] current = new GenericEntry[4];

    private ShuffleboardLayout[] driveLayouts = new ShuffleboardLayout[4];

    public SwerveTab(Drivetrain drive){
        this.drive = drive;
        modules = this.drive.getModules();
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Swerve");
        for(int i = 0; i<modules.length; i++){
            String moduleName = modules[i].getModuleType().name();
            
            driveLayouts[i] = tab.getLayout(moduleName+" swerve module", BuiltInLayouts.kList).withSize(2, 2).withPosition(2*i, 0);

            steerSpeed[i] = driveLayouts[i]
            .add("steer Speed", 0)
            .withPosition(0, 0)
            .getEntry();

            driveSpeed[i] = driveLayouts[i]
            .add("drive Speed", 0)
            .withPosition(0, 1)
            .getEntry();

            rotationalPosition[i] = driveLayouts[i]
            .add("rotation position", 0)
            .withPosition(0, 2)
            .getEntry();

            voltage[i] = driveLayouts[i]
            .add("voltage", 0)
            .withPosition(0, 3)
            .getEntry();

            current[i] = driveLayouts[i]
            .add("current", 0)
            .withPosition(0, 4)
            .getEntry();
            
            driveAccel[i] = driveLayouts[i]
            .add("drive accel", 0)
            .withPosition(0, 4)
            .getEntry();
        }

        xOdemetry = tab.add("x odemetry", 0).withPosition(0, 3).getEntry();
        yOdemetry = tab.add("y odemetry", 0).withPosition(1, 3).getEntry();
        rotOdemetry = tab.add("chassis rotation", 0).withPosition(3, 3).getEntry();
    }


    public void update(){
        for(int i = 0; i<modules.length; i++){
            steerSpeed[i].setDouble(truncate(modules[i].getSteerVelocity()));
            driveSpeed[i].setDouble(truncate(modules[i].getDriveVelocity()));
            rotationalPosition[i].setDouble(truncate(MathUtil.inputModulus(modules[i].getAngle().getDegrees(), 0, 360)));
            voltage[i].setDouble(truncate(drive.getModules()[i].getDriveVoltage()));
            current[i].setDouble(truncate(drive.getModules()[i].getDriveStatorCurrent()));
            if(RobotBase.isReal()){
                driveAccel[i].setDouble(modules[i].getDriveMotor().getAcceleration().getValueAsDouble());
            }

        }
        xOdemetry.setDouble(truncate(drive.getPose().getX()));
        yOdemetry.setDouble(truncate(drive.getPose().getY()));
        rotOdemetry.setDouble(truncate(drive.getPose().getRotation().getDegrees()));
    }
}
