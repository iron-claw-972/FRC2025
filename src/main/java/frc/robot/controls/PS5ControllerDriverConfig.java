package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.IntakeAlgae;
import frc.robot.commands.gpm.IntakeAlgaeArm;
import frc.robot.commands.gpm.IntakeCoral;
import frc.robot.commands.gpm.MoveArm;
import frc.robot.commands.gpm.MoveElevator;
import frc.robot.commands.gpm.OuttakeAlgae;
import frc.robot.commands.gpm.NetSetpoint;
import frc.robot.commands.gpm.OuttakeCoral;
import frc.robot.commands.gpm.ResetClimb;
import frc.robot.commands.gpm.StationIntake;
import frc.robot.commands.vision.AimAtGamePiece;
import frc.robot.commands.vision.DriveToGamePiece;
import frc.robot.commands.vision.LogVision;
import frc.robot.util.Vision.DetectedObject.ObjectType;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Vision.Vision;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.DPad;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    public static final boolean singleAlignmentButton = true;
    private final BooleanSupplier slowModeSupplier = () -> {return true;};
    // private final BooleanSupplier slowModeSupplier = ()->false;
    // private boolean slowMode = false;

    private final boolean autoOuttake = true;
    private BooleanSupplier menuIsOn = () -> false; 
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final Elevator elevator;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Climb climb;
    private final Arm arm;
    private final Vision vision;
    private int alignmentDirection = 0;
    private Pose2d alignmentPose = null;
    // 0 == not selected, -1 == left, 1 == right
    private byte selectedDirection = 0;

    boolean coralIntakeToggle = true;

    public PS5ControllerDriverConfig(Drivetrain drive, Elevator elevator, Intake intake, Indexer indexer, Outtake outtake, Climb climb, Arm arm, Vision vision) {
        super(drive);
        this.elevator = elevator;
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        this.climb = climb;
        this.arm = arm;
        this.vision = vision;
    }

    public void configureControls() {
        Trigger menu = driver.get(DPad.UP);
        // Elevator setpoints
        if(elevator != null && arm != null && outtake != null) {
            //L1 setpoint
            driver.get(PS5Button.CROSS).and(menu.negate()).onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(()->setAlignmentPose(false, true)),
                    new ConditionalCommand(
                        new ParallelCommandGroup(
                            new MoveElevator(elevator, ElevatorConstants.L1_SETPOINT),
                            new MoveArm(arm, ArmConstants.L1_SETPOINT),
                            new DriveToPose(getDrivetrain(), ()->alignmentPose)
                        ),
                        // idt this comment is accurate kyle
                        // This is instant so it doesn't requre the drivetrain for more than 1 frame
                        new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.L1_SETPOINT);
                            arm.setSetpoint(ArmConstants.L1_SETPOINT);
                        }),
                        ()->selectedDirection != 0
                    ),
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            new OuttakeCoral(outtake, elevator, arm),
                            new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
                            new InstantCommand(()->{
                                elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                                arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                                alignmentPose = null;
                                selectedDirection = 0;
                            }, elevator, arm)
                        ),
                        new DoNothing(),
                        () -> selectedDirection != 0 && autoOuttake
                    )
                )
            );

            driver.get(PS5Button.TRIANGLE).onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(()->setAlignmentPose(true)),
                    new ConditionalCommand(
                        new ParallelCommandGroup(
                            new MoveElevator(elevator, ElevatorConstants.L4_SETPOINT),
                            new ConditionalCommand(
                                new MoveArm(arm, ArmConstants.L4_SETPOINT_RIGHT),
                                new MoveArm(arm, ArmConstants.L4_SETPOINT_LEFT),
                                () -> selectedDirection >= 0
                            ),
                            new DriveToPose(getDrivetrain(), ()->alignmentPose)
                        ),
                        // This is instant so it doesn't requre the drivetrain for more than 1 frame
                        new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.L4_SETPOINT);
                            arm.setSetpoint(ArmConstants.L4_SETPOINT_RIGHT);
                        }),
                        ()->selectedDirection != 0
                    ),
                    new ConditionalCommand(
                        new WaitCommand(0.5),
                        new DoNothing(),
                        () -> selectedDirection < 0
                    ),
                    new ConditionalCommand(
                        new OuttakeCoral(outtake, elevator, arm)
                        .andThen(new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                            arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                            alignmentPose = null;
                            selectedDirection = 0;
                        }, elevator, arm)),
                        new DoNothing(),
                        () -> selectedDirection != 0 && autoOuttake
                    )
                )
            );

            Command l2Coral = new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(()->setAlignmentPose(false)),
                    new ConditionalCommand(
                        new ParallelCommandGroup(
                            new MoveElevator(elevator, ElevatorConstants.L2_SETPOINT),
                            new MoveArm(arm, ArmConstants.L2_L3_SETPOINT),
                            new DriveToPose(getDrivetrain(), ()->alignmentPose)
                        ),
                        // This is instant so it doesn't requre the drivetrain for more than 1 frame
                        new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.L2_SETPOINT);
                            arm.setSetpoint(ArmConstants.L2_L3_SETPOINT);
                        }),
                        ()->selectedDirection != 0
                    ),
                    new ConditionalCommand(
                        new SequentialCommandGroup(new OuttakeCoral(outtake, elevator, arm), new WaitCommand(.25))
                        .andThen(new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                            arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                            alignmentPose = null;
                            selectedDirection = 0;
                        }, elevator, arm)),
                        new DoNothing(),
                        () -> selectedDirection != 0 && autoOuttake
                    )
                )
            );
            Command l3Coral = new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(()->setAlignmentPose(false)),
                    new ConditionalCommand(
                        new ParallelCommandGroup(
                            new MoveElevator(elevator, ElevatorConstants.L3_SETPOINT),
                            new MoveArm(arm, ArmConstants.L2_L3_SETPOINT),
                            new DriveToPose(getDrivetrain(), ()->alignmentPose)
                        ),
                        // This is instant so it doesn't requre the drivetrain for more than 1 frame
                        new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.L3_SETPOINT);
                            arm.setSetpoint(ArmConstants.L2_L3_SETPOINT);
                        }),
                        ()->selectedDirection != 0
                    ),
                    new ConditionalCommand(
                        new SequentialCommandGroup(new OuttakeCoral(outtake, elevator, arm), new WaitCommand(.25))
                        .andThen(new InstantCommand(()->{
                            elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                            arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                            alignmentPose = null;
                            selectedDirection = 0;
                        }, elevator, arm)),
                        new DoNothing(),
                        () -> selectedDirection != 0 && autoOuttake
                    )
                )
            );

            Command l2Algae = new ParallelCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.BOTTOM_ALGAE_SETPOINT),
                new MoveArm(arm, ArmConstants.ALGAE_SETPOINT)).andThen(new IntakeAlgaeArm(outtake));
            Command l3Algae = new ParallelCommandGroup(
                new MoveElevator(elevator, ElevatorConstants.TOP_ALGAE_SETPOINT),
                new MoveArm(arm, ArmConstants.ALGAE_SETPOINT)).andThen(new IntakeAlgaeArm(outtake));
            
            // Not sure if menuIsOn will get set back to false because l2Algae will never end, instead I will put them into a parallel command group 
            // (it was sequential before)
            // driver.get(PS5Button.SQUARE).whileTrue(new ConditionalCommand(
            //     new ParallelCommandGroup(
            //         l2Algae, 
            //         new InstantCommand(()-> menuIsOn = ()-> false)
            //     ),
            //      new InstantCommand(l2Coral::schedule), 
            //     menuIsOn));
            // driver.get(PS5Button.CIRCLE).whileTrue(new ConditionalCommand(
            //     new ParallelCommandGroup(
            //         l3Algae,
            //         new InstantCommand(()-> menuIsOn = ()-> false)), 
            //     new InstantCommand(l3Coral::schedule), 
            //     menuIsOn));
            
            // Make so when letting go of square or circle, arm should remain in the same point 
            driver.get(PS5Button.SQUARE).whileTrue(new ConditionalCommand(
                l2Algae,
                new InstantCommand(l2Coral::schedule),
                menu
            ));
            driver.get(PS5Button.CIRCLE).whileTrue(new ConditionalCommand(
                l3Algae,
                new InstantCommand(l3Coral::schedule),
                menu
            ));

            //Processor setpoint
            driver.get(DPad.DOWN).and(menu.negate()).onTrue(
                new ParallelCommandGroup(
                    new MoveElevator(elevator, ElevatorConstants.SAFE_SETPOINT + 0.001),
                    new MoveArm(arm, ArmConstants.PROCESSOR_SETPOINT)
                )
            );

            //barge setpoint
            driver.get(PS5Button.TOUCHPAD).onTrue(new ParallelCommandGroup(
                new NetSetpoint(elevator, arm, getDrivetrain())
                // new InstantCommand(() -> slowMode = true)
            ));
        }

        // Intake/outtake

        Trigger r3 = driver.get(PS5Button.RIGHT_JOY);

        if(intake != null && indexer != null && elevator != null && outtake != null && arm != null){
            boolean toggle = true;
            Command intakeCoral = new IntakeCoral(intake, indexer, elevator, outtake, arm).deadlineFor(
                vision != null ? new AimAtGamePiece(getDrivetrain(), this, ()->vision.getBestGamePiece(Math.PI, true), ObjectType.CORAL)
                : new DoNothing());
            Command intakeAlgae = new IntakeAlgae(intake);
            driver.get(PS5Button.CROSS).onTrue(new InstantCommand(()->{
                if(r3.getAsBoolean()) return;
                if(menu.getAsBoolean()){
                    intakeAlgae.schedule();
                }else{
                    if(toggle){
                        if(intakeCoral.isScheduled()){
                            intakeCoral.cancel();
                        }else{
                            intakeCoral.schedule();
                        }
                    }else{
                        intakeCoral.schedule();
                    }
                }
            })).onFalse(new InstantCommand(()->{
                if(!toggle){
                    intakeCoral.cancel();
                }
                intakeAlgae.cancel();
            }));
            // On true, run the command to start intaking
            // On false, run the command to finish intaking if it has a coral
            Command startIntake = new StationIntake(outtake);
            // Command finishIn6take = new FinishStationIntake(intake, indexer, elevator, outtake);
            // driver.get(PS5Button.CROSS).and(r3).and(menu.negate()).onTrue(startIntake)
            //     .onFalse(new InstantCommand(()->{
            //         if(!startIntake.isScheduled()){
            //             // finishIntake.schedule();
            //         }else{
            //             startIntake.cancel();
            //         }
            // }));

            // Do we ever use this?? 
            driver.get(PS5Button.CROSS).and(r3).onTrue(
            new SequentialCommandGroup(
            new MoveElevator(elevator, ElevatorConstants.STATION_INTAKE_SETPOINT),
            new MoveArm(arm, ArmConstants.STATION_INTAKE_SETPOINT)).
            andThen(startIntake));
        }else{
            // driver.get(PS5Button.CROSS).toggleOnTrue(new DriveToCoral(()->vision.getBestGamePiece(Math.PI, true),getDrivetrain()));
            // driver.get(PS5Button.CIRCLE).toggleOnTrue(new AimAtCoral(getDrivetrain(), this, ()->vision.getBestGamePiece(Math.PI, true)));
            // driver.get(PS5Button.SQUARE).toggleOnTrue(new AimAtAlgae(getDrivetrain(), this, ()->vision.getBestGamePiece(Math.PI, true)));
            // driver.get(PS5Button.TRIANGLE).toggleOnTrue(new DriveToAlgae(()->vision.getBestGamePiece(Math.PI, true),getDrivetrain()));
            SmartDashboard.putData(new DriveToGamePiece(()->vision.getBestGamePiece(Math.PI, true),getDrivetrain(), ObjectType.CORAL));
            SmartDashboard.putData(new AimAtGamePiece(getDrivetrain(), this, ()->vision.getBestGamePiece(Math.PI, true), ObjectType.CORAL));
            SmartDashboard.putData(new DriveToGamePiece(()->vision.getBestGamePiece(Math.PI, true),getDrivetrain(), ObjectType.ALGAE));
            SmartDashboard.putData(new AimAtGamePiece(getDrivetrain(), this, ()->vision.getBestGamePiece(Math.PI, true), ObjectType.ALGAE));
            SmartDashboard.putData(new LogVision(() -> {return vision.getBestGamePiece(Math.PI, true);}));

        }
   


        if(intake != null && outtake != null && arm != null && elevator != null){
            Command algae = new SequentialCommandGroup(
                new OuttakeAlgae(outtake, intake),
                // Only move the arm and elevator in sequence when scoring in the net
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new MoveArm(arm, ArmConstants.ALGAE_STOW_SETPOINT),
                        new InstantCommand(()-> elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT)),
                        new WaitCommand(0.25),
                        new InstantCommand(()-> arm.setSetpoint(ArmConstants.INTAKE_SETPOINT))
                    ),
                    new InstantCommand(()->{
                        elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                        arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                    }),
                    // 1 meter is in between L3 and net setpoints
                    () -> elevator.getSetpoint() > 1
                )
            );
            Command coral = new OuttakeCoral(outtake, elevator, arm).alongWith(new InstantCommand(()-> getDrivetrain().setDesiredPose(()->null)))
                .andThen(
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            new MoveArm(arm, ArmConstants.INTAKE_SETPOINT), 
                            new MoveElevator(elevator, ElevatorConstants.STOW_SETPOINT), 
                            new InstantCommand(()->selectedDirection = 0)
                        ),
                        new DoNothing(),
                        // Returns true if arm is within 5 deg of start angle or L1 angle
                        ()->!arm.canMoveElevator()
                    ));
            Command cancelAlign = new InstantCommand(()->{}, getDrivetrain());

            // Coral Outtake
            Command outtakeCoral = new InstantCommand(() -> {
                if (outtake.coralLoaded()) {
                    coral.schedule();
                }
                cancelAlign.schedule();
            });

            //Right trigger - intake/outtake for coral & outtake for algae
            Command intakeCoral = new IntakeCoral(intake, indexer, elevator, outtake, arm);

            //Intake coral toggle
            Command intakeCoralToggle = new InstantCommand(() -> {
                if (coralIntakeToggle) {
                    if (intakeCoral.isScheduled()) {
                        intakeCoral.cancel();
                    } else {
                        intakeCoral.schedule();
                    }
                } else {
                    intakeCoral.schedule();
                }
            });

            // Intake/Outtake Coral 
            driver.get(PS5Button.RIGHT_TRIGGER).onTrue(new InstantCommand(() -> {
                if (!outtake.coralLoaded()) {
                    intakeCoralToggle.schedule();
                } 
                else {
                    outtakeCoral.schedule();
                }
            })).onFalse(new InstantCommand(() -> {
                if (!coralIntakeToggle) {
                    intakeCoral.cancel();
                }
            }));

            // Outtake Algae 
            driver.get(PS5Button.LEFT_TRIGGER).onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> {
                algae.schedule(); 
                }),
                new InstantCommand(() -> slowMode = false)
            ));
        }

        //Reverse Motors
        if(intake != null && indexer != null && outtake != null){
            driver.get(PS5Button.OPTIONS).and(menu.negate()).whileTrue(new SequentialCommandGroup(
                new InstantCommand(()->intake.setAngle(45)),
                new ReverseMotors(intake, indexer, outtake)
                )).onFalse(new InstantCommand(()->intake.setAngle(IntakeConstants.STOW_SETPOINT)));
        }

        // Climb
        if(climb != null){
            driver.get(PS5Button.LB).and(menu.negate()).toggleOnTrue(new StartEndCommand(()->climb.extend(), ()->climb.climb(), climb));
            if(intake != null){
                driver.get(PS5Button.LB).and(menu.negate()).onTrue(new InstantCommand(()->intake.setAngle(65), intake));
            }
            // TODO: test later, idk if this will work 
            driver.get(PS5Button.PS).and(menu).whileTrue(new SequentialCommandGroup(
                new InstantCommand(()->intake.setAngle(65), intake), 
                new ResetClimb(climb))).onFalse(new InstantCommand(()->intake.setAngle(IntakeConstants.STOW_SETPOINT)));
            driver.get(PS5Button.LB).and(menu).onTrue(new InstantCommand(()->climb.stow(), climb));
        }

        // Alignment
        driver.get(DPad.LEFT).toggleOnTrue(new InstantCommand(()->{
            selectedDirection = -1;
        }));
        driver.get(DPad.RIGHT).toggleOnTrue(new InstantCommand(()->{
            selectedDirection = 1;
        }));
        //what is this for?
        // driver.get(PS5Button.TOUCHPAD).toggleOnTrue(new InstantCommand(()->{
        //     setAlignmentPose(true, false, false, false);
        // }).andThen(new DriveToPose(getDrivetrain(), ()->alignmentPose)));

        // Reset the yaw. Mainly useful for testing/driver practice
        driver.get(PS5Button.CREATE).and(menu.negate()).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI)
        )));
        driver.get(PS5Button.CREATE).and(menu).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? Math.PI*5/6 : -Math.PI/6)
        )));

        // Cancel commands
        driver.get(PS5Button.RB).and(menu.negate()).onTrue(new InstantCommand(()->{
            if(elevator != null){
                if(outtake != null && outtake.coralLoaded()){
                    elevator.setSetpoint(ElevatorConstants.INTAKE_STOW_SETPOINT);
                }else{
                    elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                }
            }
            if(outtake != null){
                outtake.stop();
            }
            if(intake != null){
                intake.stow();
                intake.deactivate();
            }
            if(indexer != null){
                indexer.stop();
            }
            if(climb != null){
                climb.stow();
            }
            if(arm != null){
                if(outtake != null && outtake.coralLoaded()){
                    arm.setSetpoint(ArmConstants.STOW_SETPOINT);
                }else{
                    arm.setSetpoint(ArmConstants.START_ANGLE);
                }
            }
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(()->null);
            alignmentPose = null;
            selectedDirection = 0;
            CommandScheduler.getInstance().cancelAll();
        }));
    
        //Straighten wheels
        driver.get(PS5Button.MUTE).and(menu).onTrue(new FunctionalCommand(
            ()->getDrivetrain().setStateDeadband(false),
            getDrivetrain()::alignWheels,
            interrupted->getDrivetrain().setStateDeadband(true),
            ()->false, getDrivetrain()).withTimeout(2));

        //Slow mode
        // driver.get(PS5Button.TOUCHPAD).toggleOnTrue(
        //     new InstantCommand(() -> slowMode = !slowMode)
        // );

        // Only use this if you want TOGGLE for menu 
        // driver.get(DPad.UP).whileTrue(new InstantCommand(() -> menuIsOn = () -> false));
    }

    /**
     * Sets the drivetrain's alignmetn pose to the nearest reef branch or algae location
     * @param isAlgae True for algae, false for branches
     * @param isTrue for left branch, false for right, ignored for algae
     * @param l4 If the robot should align to the L4 scoring pose
     */
    private void setAlignmentPose(boolean isAlgae, boolean isLeft, boolean l4, boolean l1){
        Translation2d drivePose = getDrivetrain().getPose().getTranslation();
        int closestId = 0;
        double closestDist = 20;
        boolean isRed = Robot.getAlliance() == Alliance.Red;
        int start = isRed ? 5 : 16;
        for(int i = 0; i < 6; i++){
            double dist = FieldConstants.APRIL_TAGS.get(start+i).pose.toPose2d().getTranslation().getDistance(drivePose);
            if(dist < closestDist){
                closestDist = dist;
                closestId = start+i+1;
            }
        }
        if(isAlgae){
            alignmentPose = VisionConstants.REEF.fromAprilTagIdAlgae(closestId).pose;
        }else{
            if(l4){
                alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(closestId, isLeft).l4Pose;
            }else if(l1){
                alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(closestId, isLeft).l1Pose;
            }else{
                alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(closestId, isLeft).pose;
            }
        }
    }

    /**
     * Sets the drivetrain's alignmetn pose to the nearest reef branch with the selected direction
     * @param l4 If the robot should align to the L4 scoring pose
     * @param l1 If the robot should align to the L1 scoring pose
     */
    private void setAlignmentPose(boolean l4, boolean l1){
        if(selectedDirection == 0){
            alignmentPose = null;
            return;
        }
        setAlignmentPose(false, selectedDirection < 0, l4, l1);
    }
    /**
     * Sets the drivetrain's alignmetn pose to the nearest reef branch with the selected direction
     * @param l4 If the robot should align to the L4 scoring pose
     */
    private void setAlignmentPose(boolean l4){
        setAlignmentPose(l4, false);
    }
    
    @Override
    public double getRawSideTranslation() {
        return driver.get(PS5Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return driver.get(PS5Axis.LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return driver.get(PS5Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(driver.get(PS5Axis.RIGHT_X), -driver.get(PS5Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return Math.hypot(driver.get(PS5Axis.RIGHT_X), driver.get(PS5Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        //return slowMode;
        return false;
    }

    @Override
    public boolean getIsAlign() {
        return false;
    }

    public void startRumble(){
        driver.rumbleOn();
    }

    public void endRumble(){
        driver.rumbleOff();
    }
}
