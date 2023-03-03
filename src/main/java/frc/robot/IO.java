package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.Button;

import static frc.robot.RobotMap.*;


public class IO {
    private static Joystick driver = new Joystick(DRIVE_PORT);
    private static XboxController liftCont = new XboxController(LIFT_PORT);

    public static boolean getButton(int btn) {
        return driver.getRawButtonPressed(btn);
    }
    
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

    public static double getPOV() {
        // return liftCont.getLeftTriggerAxis() - liftCont.getRightTriggerAxis();
        return driver.getPOV();
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
