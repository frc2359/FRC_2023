package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.Button;



import static frc.robot.RobotMap.*;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;


public class IO {
    /* ------------------------------- CONTORLLER ------------------------------- */
    private static Joystick driver = new Joystick(OIConstants.DRIVE_PORT);
    private static XboxController liftCont = new XboxController(OIConstants.LIFT_PORT);

    /* -------------------------------- LIMELIGHT ------------------------------- */
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static NetworkTableEntry ty = limelightTable.getEntry("ty");
    private static NetworkTableEntry tz = limelightTable.getEntry("tz");
    private static NetworkTableEntry tarea = limelightTable.getEntry("ta");

    /* ---------------------------------- GYRO ---------------------------------- */
    private static final AHRS navx = new AHRS(SPI.Port.kMXP);
    private static final ADXRS450_Gyro adx = new ADXRS450_Gyro();





    /* -------------------------------------------------------------------------- */
    /*                                    GYRO                                    */
    /* -------------------------------------------------------------------------- */
    
    public static class GyroType {
        public static final boolean kNAVX = false;
        public static final boolean kADXRS = true;
    }

    public static void zeroHeading() {
        navx.reset();
        adx.reset();
    }

    public static boolean isRotating(int gyroType) {
        return navx.isRotating();
    }

    public static double getPitch() {
        return navx.getPitch();
    }

    public static double getRoll() {
        return navx.getPitch();
    }

    public static double getAngle(boolean gyroType) {
        return (gyroType == GyroType.kNAVX ? navx : adx).getAngle();
    }

    public static double getYaw(boolean gyroType) {
        if (gyroType == GyroType.kNAVX) {
            return navx.getYaw();
        } else {
            return (adx.getAngle() % 360) - 180;
        }
    }

    public static Rotation2d getRotation2D(boolean gyroType) {
        return (gyroType == GyroType.kNAVX ? navx : adx).getRotation2d();
    }


    /* -------------------------------------------------------------------------- */
    /*                                  LIMELIGHT                                 */
    /* -------------------------------------------------------------------------- */

    /**Returns a hashmap of the numbers returned from the limelight
     * @return Hashmap of entries for x ("tx"), y ("ty"), z ("tz"), and area ("tarea").
     */
    public static HashMap<String, Double> getLimelightValues() {
        HashMap<String, Double> m = new HashMap<String, Double>();
        m.put("tx", tx.getDouble(0.0));
        m.put("ty", ty.getDouble(0.0));
        m.put("tz", tz.getDouble(0.0));
        m.put("tarea", tarea.getDouble(0.0));
        return m;
    }

    /* -------------------------------------------------------------------------- */
    /*                               OPERATOR INPUT                               */
    /* -------------------------------------------------------------------------- */

    /**Checks Button <b>FOR THE DRIVE CONTROLLER</b> 
     * @param btn is the targeted button
    */
    public static boolean getButton(int btn) {
        return driver.getRawButtonPressed(btn);
    }

    /**Get selected axis
     * @param ax is the axis you selected
     */
    public static double getRawAxis(int ax) {
        return driver.getRawAxis(ax);
    }

    /**Get the lower dial, values from -1 to 1 */
    public static double getSpeedDial() {
        return driver.getRawAxis(3);
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

    /**Checks POV <b>FOR THE LIFT CONTROLLER</b> */
    public static int getLiftPOV() {
        return liftCont.getPOV();
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
