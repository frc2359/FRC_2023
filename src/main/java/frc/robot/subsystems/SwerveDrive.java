package frc.robot.subsystems;

import static frc.robot.RobotMap.*;

import java.lang.Math;

import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

// EXAMPLE CODE: https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/develop/examples/mk3-testchassis/src/main/java/com/swervedrivespecialties/examples/mk3testchassis/subsystems/DrivetrainSubsystem.java

public class SwerveDrive {
        private SwerveModule frontLeftModule;
        private SwerveModule frontRightModule;
        private SwerveModule backLeftModule;
        private SwerveModule backRightModule;

        private static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public SwerveDrive() {
                frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                        Mk4iSwerveModuleHelper.GearRatio.L1,
                        FRONT_LEFT_DRIVE,
                        FRONT_LEFT_STEER,
                        FRONT_LEFT_ENC,
                        FRONT_LEFT_MODULE_STEER_OFFSET
                );

        }
}
