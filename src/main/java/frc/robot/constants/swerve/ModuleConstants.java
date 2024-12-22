package frc.robot.constants.swerve;

import frc.robot.constants.IdConstants;

/**
 * Container class for module constants, defined using constants from {@link DriveConstants}
 * .
 *
 * @see DriveConstants
 */
public enum ModuleConstants {

    FRONT_LEFT(
            IdConstants.DRIVE_FRONT_LEFT_ID,
            IdConstants.STEER_FRONT_LEFT_ID,
            IdConstants.ENCODER_FRONT_LEFT_ID,
            DriveConstants.STEER_OFFSET_FRONT_LEFT,
            ModuleType.FRONT_LEFT,
            DriveConstants.S_VALUES[0],
            DriveConstants.V_VALUES[0],
            DriveConstants.A_VALUES[0],
            DriveConstants.P_VALUES[0],
            DriveConstants.I_VALUES[0],
            DriveConstants.D_VALUES[0]
    ),
    FRONT_RIGHT(
            IdConstants.DRIVE_FRONT_RIGHT_ID,
            IdConstants.STEER_FRONT_RIGHT_ID,
            IdConstants.ENCODER_FRONT_RIGHT_ID,
            DriveConstants.STEER_OFFSET_FRONT_RIGHT,
            ModuleType.FRONT_RIGHT,
            DriveConstants.S_VALUES[1],
            DriveConstants.V_VALUES[1],
            DriveConstants.A_VALUES[1],
            DriveConstants.P_VALUES[1],
            DriveConstants.I_VALUES[1],
            DriveConstants.D_VALUES[1]
    ),
    BACK_LEFT(
            IdConstants.DRIVE_BACK_LEFT_ID,
            IdConstants.STEER_BACK_LEFT_ID,
            IdConstants.ENCODER_BACK_LEFT_ID,
            DriveConstants.STEER_OFFSET_BACK_LEFT,
            ModuleType.BACK_LEFT,
            DriveConstants.S_VALUES[2],
            DriveConstants.V_VALUES[2],
            DriveConstants.A_VALUES[2],
            DriveConstants.P_VALUES[2],
            DriveConstants.I_VALUES[2],
            DriveConstants.D_VALUES[2]
    ),
    BACK_RIGHT(
            IdConstants.DRIVE_BACK_RIGHT_ID,
            IdConstants.STEER_BACK_RIGHT_ID,
            IdConstants.ENCODER_BACK_RIGHT_ID,
            DriveConstants.STEER_OFFSET_BACK_RIGHT,
            ModuleType.BACK_RIGHT,
            DriveConstants.S_VALUES[3],
            DriveConstants.V_VALUES[3],
            DriveConstants.A_VALUES[3],
            DriveConstants.P_VALUES[3],
            DriveConstants.I_VALUES[3],
            DriveConstants.D_VALUES[3]
    ),

    NONE(0, 0, 0, 0.0, ModuleType.NONE,0,0,0,0,0,0);

    private final int drivePort;
    private final int steerPort;
    private final int encoderPort;
    private final double steerOffset;
    private final double ks;
    private final double kv;
    private final double ka;
    private final double driveP;
    private final double driveI;
    private final double driveD;
    private final ModuleType type;

    ModuleConstants(
            int drivePort,
            int steerPort,
            int encoderPort,
            double steerOffset,
            ModuleType type,
            double ks,
            double kv,
            double ka,
            double driveP,
            double driveI,
            double driveD

                   ) {
        this.drivePort = drivePort;
        this.steerPort = steerPort;
        this.encoderPort = encoderPort;
        this.steerOffset = steerOffset;
        this.type = type;
        this.ks =ks;
        this.kv= kv;
        this.ka = ka;
        this.driveP =driveP;
        this.driveI = driveI;
        this.driveD = driveD;
    }

    public int getDrivePort() {
        return drivePort;
    }

    public int getSteerPort() {
        return steerPort;
    }

    public int getEncoderPort() {
        return encoderPort;
    }

    public double getSteerOffset() {
        return steerOffset;
    }

    public ModuleType getType() {
        return type;
    }
    public double getDriveS(){
        return ks;
    }
    public double getDriveV(){
        return kv;
    }
    public double getDriveA(){
        return ka;
    }
    public double getDriveP(){
        return driveP;
    }
    public double getDriveI(){
        return driveI;
    }
    public double getDriveD(){
        return driveD;
    }

}