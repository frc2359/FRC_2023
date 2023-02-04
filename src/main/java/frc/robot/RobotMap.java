package frc.robot;

//import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface RobotMap {

    /* ---------------------------------- IO & HANDLING --------------------------------- */
    public static final int DRIVE_PORT = 1; //USB IO Port  -- joystick init in robotcontainer, too.  Swith if needed
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
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_STEER = 5;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int FRONT_RIGHT_STEER = 6;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_LEFT_STEER = 7;
    public static final int BACK_RIGHT_DRIVE = 4;
    public static final int BACK_RIGHT_STEER = 8;

    /* -------------------------- SWERVE ENCODER PORTS -------------------------- */
    public static final int FRONT_LEFT_ENC = 1;
    public static final int FRONT_RIGHT_ENC = 2;
    public static final int BACK_LEFT_ENC = 3;
    public static final int BACK_RIGHT_ENC = 4;

    /* -------------------------- SWERVE STEER OFFSETS -------------------------- */
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(362);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-145);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(125);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-90);

    /* ----------------------------- DRIVE CONSTANTS ---------------------------- */
    public static final int DRIVE_TICKS_TO_ROTATION = 1250;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 24.0; // front left to front right
    public static final double DRIVETRAIN_WHEELBASE_METERS = 25.3;  // front left to back left


    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio =  (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);  // check
        public static final double kTurningMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);  // check
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(16.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // front right
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),     // front left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),   // rear right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));   // rear left

        //public static final int kFrontLeftDriveMotorPort = 1;
        //public static final int kBackLeftDriveMotorPort = 3;
        //public static final int kFrontRightDriveMotorPort = 2;
        //public static final int kBackRightDriveMotorPort = 4;
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 1;

        //public static final int kFrontLeftTurningMotorPort = 5;
        //public static final int kBackLeftTurningMotorPort = 7;
        //public static final int kFrontRightTurningMotorPort = 6;
        //public static final int kBackRightTurningMotorPort = 8;
        public static final int kFrontLeftTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 5;


        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        //public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        //public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        //public static final int kBackRightDriveAbsoluteEncoderPort = 4;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(15);   //345 deg
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -Math.toRadians(55);     // 55 deg
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(37);   // 323 deg
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(84);    // 276 deg

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }
}
