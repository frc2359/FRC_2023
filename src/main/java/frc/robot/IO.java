package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.Button;

import static frc.robot.RobotMap.*;


public class IO {
    private static Joystick driver = new Joystick(DRIVE_PORT);
    private static XboxController liftCont = new XboxController(LIFT_PORT);

    /**Checks Button <b>FOR THE DRIVE CONTROLLER</b> 
     * @param btn is the targeted button
    */
    public static boolean getButton(int btn) {
        return driver.getRawButtonPressed(btn);
    }

    /**Checks Left Y Axis <b>FOR THE LIFT CONTROLLER</b> */
    public static double getLiftControlLeftY() {
        return liftCont.getLeftY();
    }
    
    /**Checks Left X Axis <b>FOR THE LIFT CONTROLLER</b> */
    public static double getLiftControlLeftX() {
        return liftCont.getLeftX();
    }

    public static double getLiftControlRightX() {
        return liftCont.getRightX();
    }

    /**Checks X <b>FOR THE LIFT CONTROLLER</b> */
    public static boolean isXPressed() {
        return liftCont.getXButtonPressed();
    }

    /**Checks Y <b>FOR THE LIFT CONTROLLER</b> */
    public static boolean isYPressed() {
        return liftCont.getYButtonPressed();
    }

    /**Checks A <b>FOR THE LIFT CONTROLLER</b> */
    public static boolean isAPressed() {
        return liftCont.getAButtonPressed();
    }

    /**Checks B <b>FOR THE LIFT CONTROLLER</b> */
    public static boolean isBPressed() {
        return liftCont.getBButtonPressed();
    }
    
    /**Checks X Axis <b>FOR THE DRIVE CONTROLLER</b> */
    public static double getDriveX() {
        return Math.abs(driver.getX()) > 0.1 ? driver.getX() : 0;
    }

    /**Checks Y Axis <b>FOR THE DRIVE CONTROLLER</b> */
    public static double getDriveY() {
        return Math.abs(driver.getY()) > 0.1 ? driver.getY() : 0;
    }

    /**Checks stick angle <b>FOR THE DRIVE CONTROLLER</b> */
    public static double getDriveDirection() {
        return driver.getDirectionRadians();
    }

    /**Checks stick magnitude <b>FOR THE DRIVE CONTROLLER</b> */
    public static double getDriveMagnitude() {
        return driver.getMagnitude();
    }

    /**Checks stick twist <b>FOR THE DRIVE CONTROLLER</b> */
    public static double getDriveTwist() {
        SmartDashboard.putNumber("Twist", driver.getTwist());
        return Math.abs(driver.getTwist()) > 0.5 ? driver.getTwist() * 0.5 : 0;
    }

    /**Checks POV (little hat guy on top) <b>FOR THE DRIVE CONTROLLER</b> */
    public static double getPOV() {
        // return liftCont.getLeftTriggerAxis() - liftCont.getRightTriggerAxis();
        return driver.getPOV();
    }

    /**Checks if POV (little hat guy on top) is rotated to an angle <b>FOR THE DRIVE CONTROLLER</b>
     * @param angle is the desired angle to check for
     */
    public static boolean isPOVToAngle(double angle) {
        return driver.getPOV() == angle;
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
