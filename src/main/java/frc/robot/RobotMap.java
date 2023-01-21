package frc.robot;

public interface RobotMap {
    public static final int GRIPPER_0 = 8;
    public static final int GRIPPER_1 = 9;
    public static final int GRIPPER_2 = 10;

    // drive
    public static final int DRIVE_PORT = 0; //USB IO Port
    public static final double TURN_SPEED_MULT = 1;
    public static final double DRIVE_SPEED_MULT = 1;

    public static final boolean BRAKE_MODE_DRIVE = true;

    public static final int DRIVE_1 = 1;
    public static final int DRIVE_2 = 2;
    public static final int DRIVE_3 = 3;
    public static final int DRIVE_4 = 4;

    // Swerve motors
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_STEER = 2;
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_STEER = 4;
    public static final int BACK_LEFT_DRIVE = 5;
    public static final int BACK_LEFT_STEER = 6;
    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_STEER = 8;

    //Swerve encoder
    public static final int FRONT_LEFT_ENC = 1;
    public static final int FRONT_RIGHT_ENC = 2;
    public static final int BACK_LEFT_ENC = 3;
    public static final int BACK_RIGHT_ENC = 4;

    // steer offset
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);

    // drive constants (measurements)
    public static final int DRIVE_TICKS_TO_ROTATION = 1250;
    public static final int DRIVETRAIN_TRACKWIDTH_METERS = 1;
    public static final int DRIVETRAIN_WHEELBASE_METERS = 1;

}
