package frc.robot;

//import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface RobotMap {

    /* ---------------------------------- IO & HANDLING --------------------------------- */
       

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

    /* ----------------------------- COMMAND BUTTONS ---------------------------- */
    public static final int CMD_BUTTON_HOME = 1; //
    public static final int CMD_BUTTON_DOUBLE = 2; //
    public static final int CMD_BUTTON_CHUTE = 3; //
    public static final int CMD_BUTTON_GROUND = 4; //
    public static final int CMD_BUTTON_INTAKE = 5; //
    public static final int CMD_BUTTON_STOP = 6; //
    public static final int CMD_BUTTON_EXP_SL = 7; //
    public static final int CMD_BUTTON_EXP_FA = 8; //
    public static final int CMD_BUTTON_LOW = 9;
    public static final int CMD_BUTTON_CU_HIGH = 10;
    public static final int CMD_BUTTON_CU_MID = 11; //
    public static final int CMD_BUTTON_CO_MID = 12;

    /* ----------------------------- SETTING BUTTONS ---------------------------- */
    public static final int SETTING_BUTTON_0 = 0;
    public static final int SETTING_BUTTON_1 = 1;
    public static final int SETTING_BUTTON_2 = 2;



    public static final class LimelightConsants {
        public static final int kPipelineLedSettings = 0;
        public static final int kLedOff = 1;
        public static final int kLedBlink = 2;
        public static final int kLedOn = 3;
    }

    public static final class ModuleConstants {
        public static final GearRatio SWERVE_GEAR_RATIO = Mk4iSwerveModuleHelper.GearRatio.L1; //OLD


        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio =  (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);  // check
        public static final double kTurningMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);  // check
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kPDriving = 0.5;
    }

    public static final class DriveConstants {
        public static final boolean INI_BRAKE_MODE_DRIVE = true;

        public static final boolean kBrakeMode = true;
        public static final boolean kCoastMode = false;

        public static boolean currentBrakeMode = INI_BRAKE_MODE_DRIVE;

        /* -------------------------------------------------------------------------- */
        /*                             PHYSICAL CONSTANTS                             */
        /* -------------------------------------------------------------------------- */

        // set Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(16.5);
        
        // set Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(24.5);
        
        // set location of each module in relation to the center
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),    // front right
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),     // front left
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   // rear right
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));   // rear left
    
        /* -------------------------------------------------------------------------- */
        /*                            DRIVETRAIN CONSTANTS                            */
        /* -------------------------------------------------------------------------- */

        // MOTOR CONSTANTS ----------------------------------------------------------
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 1;

        
        public static final int kFrontLeftTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 5;


        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // ENCODER CONSTANTS ---------------------------------------------------------
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed =false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(180);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(180);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(179);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(180);
        
        // SPEED CONSTANTS ---------------------------------------------------------
        public static final double kPhysicalMaxSpeedMetersPerSecond = 8;  // 13.5?
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond - 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 7;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final int MAX_PATH_SPEED_AUTO = 3;
        public static final int MAX_PATH_ACCEL_AUTO = 2;

        public static final int kOut = 1;
        //all after this will run the initial cube stuff
        public static final int kShootAndFarOutSpin = 2;
        public static final int kTwoCubes = 3;
        public static final int kShootOutBalance = 4;

        public static final String kCubeHigh = "kCubeHigh";
        public static final String kCubeMid = "kCubeMid";


        public static final int UNKNOWN_HOME = 0;
        public static final int AUTO_STATE_LIFTER_DOWN = 1;
        public static final int AUTO_STATE_SHOOT_HIGH = 2;
        public static final int AUTO_STATE_HOME_LIFTER = 3;
        public static final int AUTO_STATE_MOVE_OUT = 4;
        public static final int AUTO_STATE_MOVING_OUT = 5;
        public static final int AUTO_STATE_LIFTER_CUBE = 6;
        public static final int AUTO_STATE_INTAKE_CUBE = 7;
        public static final int AUTO_STATE_RETURN_CUBE = 8;
        public static final int AUTO_STATE_RETURN_ZERO = 9;
        public static final int AUTO_STATE_RETURN_LIFTER_DOWN = 10;
        public static final int AUTO_STATE_RETURN_SHOOT = 11;
        public static final int AUTO_STATE_FINISH = 12;

        public static final int kZero = 1;
        public static final int kDown = 2;
        public static final int kMidCube = 3;
        public static final int kHighCube = 4;
        public static final int kMidCone = 5;
        public static final int kHighCone = 6;
        public static final int kChute = 7;
        public static final int kSubstation = 8;
        public static final int kCollectLow = 9;
        public static final int kLow = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        /* -------------------------------------------------------------------------- */
        /*                   DRIVE CONSTANTS - VELOCITY MOTION MAGIC                  */
        /* -------------------------------------------------------------------------- */
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /**
         * Gains used in Motion Magic, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

        
    }

    /**
     *  Class that organizes gains used when assigning values to slots
     */
    public static final class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int kIzone;
        public final double kPeakOutput;
        
        public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = _kIzone;
            kPeakOutput = _kPeakOutput;
        }
    }

    public static final class OIConstants {
        public static final int DRIVE_PORT = 0; //USB IO Port  -- joystick init in robotcontainer, too.  Swith if needed
        public static final int LIFT_PORT = 1;
        public static final int BOX_PORT = 2;

        public static final boolean SEPARATE_CONTROLS = true;

        public static final double TURN_SPEED_MULT = 1;
        public static final double DRIVE_SPEED_MULT = 1;

        public static final double kDriverDeadband = 0.2;


        // public static final int kDriverControllerPort = 0;

        // public static final int kDriverYAxis = 1;
        // public static final int kDriverXAxis = 0;
        // public static final int kDriverRotAxis = 2;
        // public static final int kDriverFieldOrientedButtonIdx = 1;

    }

    public static final class LifterConstants {
        public static final double kRampRateSec = 1;

        public static final double kMaxVoltage = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.2;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 1.2;

        public static final double kMaxMultiplier = 0.7;

        public static final int LIFT_ID = 10;
        public static final int EXTEND_ID = 11;
        public static final int CLAW_ID = 12;
        public static final int LIFT_LIMIT = 3;

        public static final int LIFTER_MAX_ROTATION = 120;

        public static final int STATE_LIFT_UNKOWN = -1;
        public static final int STATE_LIFT_STOP = 0;
        public static final int STATE_LIFT_ZERO_ENCODERS = 1;
        public static final int STATE_LIFT_UP = 2;
        public static final int STATE_LIFT_DOWN = 3;
        public static final int STATE_LIFT__MOVE_TO_POS = 4;

        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        //         new TrapezoidProfile.Constraints(
        //                 kMaxAngularSpeedRadiansPerSecond,
        //                 kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ClawConstants {
        public static final int CLAW_MOT_ID = 12;

        public static final int CLAW_LEFT_MOT_ID = 12;
        public static final int CLAW_RIGHT_MOT_ID = 13;

        public static final int HAS_CONE_ID = 4;
        

        /* -------------------------------- OLD CASES ------------------------------- */
        // public static final int CASE_STOP = 0;
        // public static final int CASE_OPEN = 1;
        // public static final int CASE_CLOSED = 2;

        /* ---------------------------------- CASES --------------------------------- */
        public static final int CASE_CO_WAIT = -1;
        public static final int CASE_STOP = 0;
        
        public static final int CASE_INTAKE = 1;
        public static final int CASE_POWERED_HOLD = 2;
        public static final int CASE_EXPEL_CUBE_LOW = 3;
        public static final int CASE_EXPEL_CUBE_MID = 4;
        public static final int CASE_EXPEL_CUBE_HIGH = 5;
        public static final int CASE_EXPEL_CONE = 6;

    }

    public static final class ExtenderConstants {
        public static double currentlyExtended = 0; //NOT A CONST

        public static final int EXTEND_MOT_ID = 14;

        /* ---------------------------------- CASES --------------------------------- */
        public static final int CASE_ZERO_ENCODERS = -1;
        public static final int CASE_STOP = 0;
        public static final int CASE_UP = 1;
        public static final int CASE_DOWN = 2;
        public static final int CASE_EXTEND_TO_DIST = 3;
        public static final int CASE_UP_SLOW = 4;
        public static final int CASE_DOWN_SLOW = 5;

        /* ------------------------- EXTENDER STATES -------------------------------- */
        public static final int STATE_EXT_UNKNOWN = -1;
        public static final int STATE_EXT_STOP = 0;
        public static final int STATE_EXT_EXTEND = 1;
        public static final int STATE_EXT_RETRACT = 2;
        public static final int STATE_EXT_MOVE_TO_POS = 9;

        /* ------------------------- EXTENDER CONSTANTS ----------------------------- */
        public static final double EXTENDER_MAX_DISTANCE = 17.5;   // 18.5 inches
        public static final double EXTENDER_SLOW_DISTANCE = 1.0;     // 2 inches
        public static final double EXTENDER_FAST_SPEED = 1;
        public static final double EXTENDER_SLOW_SPEED = 0.7;
    }

    public static final class LEDConstants {
        public static final int PWM_LED_LEFT = 0;
        public static final int PWM_LED_RIGHT = 1;

    }

}
