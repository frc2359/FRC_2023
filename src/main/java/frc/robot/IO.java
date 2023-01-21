package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.RobotMap.*;


public class IO {
    private static Joystick driver = new Joystick(DRIVE_PORT);

    public static double getDriveX() {
        return driver.getX();
    }

    public static double getDriveY() {
        return driver.getY();
    }

    public static double getDriveDirection() {
        return driver.getDirectionRadians();
    }

    public static double getDriveMagnitude() {
        return driver.getMagnitude();
    }

    public static double getDriveTwist() {
        return driver.getTwist();
    }

    private final SwerveDrive drivetrain = new SwerveDrive();

    public void SwerveDrive() {
        drivetrain.register();

        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -modifyAxis(getDriveY()), // Axes are flipped here on purpose
                () -> -modifyAxis(getDriveX()),
                () -> -modifyAxis(getDriveTwist())
        ));

        // new Button(driver::getTrigger)
        //         .whenPressed(drivetrain::zeroGyroscope);
    }

    public SwerveDrive getDrivetrain() {
        return drivetrain;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
