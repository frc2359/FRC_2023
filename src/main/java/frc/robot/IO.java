package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
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
}
