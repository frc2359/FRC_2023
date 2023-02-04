package frc.robot;

import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

public interface RobotMap {

    /* ---------------------------------- IO & HANDLING --------------------------------- */
    public static final int DRIVE_PORT = 0; //USB IO Port
    public static final double TURN_SPEED_MULT = 1;
    public static final double DRIVE_SPEED_MULT = 1;

    public static final boolean BRAKE_MODE_DRIVE = true;



    /* -------------------------------------------------------------------------- */
    /*                                   GRIPPER                                  */
    /* -------------------------------------------------------------------------- */

    /* ---------------------------- GRIPPER MOTOR ID ---------------------------- */
    public static final int GRIPPER_0 = 8;
    public static final int GRIPPER_1 = 9;
    public static final int GRIPPER_2 = 10;

    

    /* -------------------------------------------------------------------------- */
    /*                                 BASIC DRIVE                                */
    /* -------------------------------------------------------------------------- */
    public static final int DRIVE_1 = 1;
    public static final int DRIVE_2 = 2;
    public static final int DRIVE_3 = 3;
    public static final int DRIVE_4 = 4;



    /* -------------------------------------------------------------------------- */
    /*                                SWERVE DRIVE                                */
    /* -------------------------------------------------------------------------- */

    public static final GearRatio SWERVE_GEAR_RATIO = Mk4iSwerveModuleHelper.GearRatio.L1;

    /* --------------------------- SWERVE MOTOR ID --------------------------- */
    public static final int FRONT_LEFT_DRIVE = 4;
    public static final int FRONT_LEFT_STEER = 8;
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_STEER = 7;
    public static final int BACK_LEFT_DRIVE = 2;
    public static final int BACK_LEFT_STEER = 6;
    public static final int BACK_RIGHT_DRIVE = 1;
    public static final int BACK_RIGHT_STEER = 5;

    /* -------------------------- SWERVE ENCODER PORTS -------------------------- */
    public static final int FRONT_LEFT_ENC = 1;
    public static final int FRONT_RIGHT_ENC = 2;
    public static final int BACK_LEFT_ENC = 3;
    public static final int BACK_RIGHT_ENC = 4;

    /* -------------------------- SWERVE STEER OFFSETS -------------------------- */
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(362);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(215);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(125);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-90);

    /* ----------------------------- DRIVE CONSTANTS ---------------------------- */
    public static final int DRIVE_TICKS_TO_ROTATION = 1250;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 24.0; // front left to front right
    public static final double DRIVETRAIN_WHEELBASE_METERS = 25.3;  // front left to back left
}
