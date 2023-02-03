package frc.robot.subsystems;

import static frc.robot.RobotMap.*;

import java.lang.Math;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.sensors.PigeonIMU; // REPLACE WITH NAVX
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

// EXAMPLE CODE: https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/develop/examples/mk3-testchassis/src/main/java/com/swervedrivespecialties/examples/mk3testchassis/subsystems/DrivetrainSubsystem.java

public class SwerveDrive extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


    // initializing the wheels

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    // creating a gyroscope object (NavX)

    private final AHRS gyroscope = new AHRS(SPI.Port.kMXP);

    // setting wheel position on the robot (FR, FL, RR, RL)
    // * positive x is pointing away from the driver's station
    // * positive y is pointing perpendicular to x
    // The angle between x and y is measured in a counter-clockwise direction
    // From the top down: positive x would be forward, positive y would be to the left
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / -2.0, DRIVETRAIN_WHEELBASE_METERS / -2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / -2.0));

    // initialize odometry

    private final SwerveDriveOdometry odometry;

    // what the speed of each wheel should be

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // method that creates a combination of falcon and neo motors being used together

    public SwerveDrive() {

        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                SWERVE_GEAR_RATIO,
                FRONT_LEFT_DRIVE,
                FRONT_LEFT_STEER,
                FRONT_LEFT_ENC,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                SWERVE_GEAR_RATIO,
                FRONT_RIGHT_DRIVE,
                FRONT_RIGHT_STEER,
                FRONT_RIGHT_ENC,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                SWERVE_GEAR_RATIO,
                BACK_LEFT_DRIVE,
                BACK_LEFT_STEER,
                BACK_LEFT_ENC,
                BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                SWERVE_GEAR_RATIO,
                BACK_RIGHT_DRIVE,
                BACK_RIGHT_STEER,
                BACK_RIGHT_ENC,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        odometry = new SwerveDriveOdometry(kinematics,
            Rotation2d.fromDegrees(gyroscope.getFusedHeading()), new SwerveModulePosition[] {
                new SwerveModulePosition(frontLeftModule.getDriveVelocity(),
                        new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(frontRightModule.getDriveVelocity(),
                        new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModulePosition(backLeftModule.getDriveVelocity(),
                        new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModulePosition(backRightModule.getDriveVelocity(),
                        new Rotation2d(backRightModule.getSteerAngle())) });

    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro", gyroscope.getFusedHeading());
       
        odometry.update(Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(frontLeftModule.getDriveVelocity(),
                                new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getDriveVelocity(),
                                new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getDriveVelocity(),
                                new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getDriveVelocity(),
                                new Rotation2d(backRightModule.getSteerAngle())) });

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }
}
