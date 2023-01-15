package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.IO;
import frc.robot.IO.*;
import static frc.robot.RobotMap.*;

public class SimpleDrive {

    //wheels
    public final VictorSP drive0 = new VictorSP(0);
    public final VictorSP drive1 = new VictorSP(1);
    public final VictorSP drive2 = new VictorSP(2);
    public final VictorSP drive3 = new VictorSP(3);

    //joystick
    public final Joystick joy = new Joystick(0);

    //encoder
    public final Encoder encoder0 = new Encoder(0, 1);

    //differential drive splitting
    private DifferentialDrive diffDrive0 = new DifferentialDrive(drive0, drive1);
    private DifferentialDrive diffDrive1 = new DifferentialDrive(drive2, drive3);

    //PID UNKNOWNS
    private double error = 0;
    private double errorSum = 0;
    private double errorRate = 0;

    //PID KNOWNS (HARDCODE AFTER TRIAL AND ERROR)
    //THESE VALUES WILL CHANGE IF THE DIMENSIONS OF THE ROBOT CHANGE
    private double kP = 0.0001;
    private double kI = 0.0001;
    private double kD = 0.0001;

    //TIMER
    private double lastTimestamp = 0;

    //initialize safety for wheels
    public void init() {
        drive0.setSafetyEnabled(true);
        drive1.setSafetyEnabled(true);
        drive2.setSafetyEnabled(true);
        drive3.setSafetyEnabled(true);

        // drive0.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        // drive1.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        // drive2.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        // drive3.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);

        //ALL UNITS IN INCHES

        //wheel numbers for FALCON
        double falconDegreesPerPulse = 360/2048;
        double falconRadius = 1.75;

        //wheel numbers for TEST WHEELS (THESE ARE ROUGH ESTIMATES DO NOT EXPECT ACCURACY)
        double testDegreesPerPulse = 360/360;
        double testRadius = 2.8;

        //REAL wheel numbers (CHANGE THESE WHEN SWITCHING BETWEEN ROBOTS OR THE ROBOT IS GONNA RUN AWAY AND NEVER COME BACK)
        double realDegreesPerPulse = testDegreesPerPulse;
        double realRadius = testRadius;

        //set encoder distance
        double circumference = 2 * Math.PI * realRadius;
        double distancePerPulse = circumference / realDegreesPerPulse;
        encoder0.setDistancePerPulse(distancePerPulse);
    }

    //set PID values for testing
    public void setPIDValues(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    //sets all motor outputs to same value
    public void setMotors(double val) {
        drive0.set(val);
        drive1.set(val);
        drive2.set(val);
        drive3.set(val);
    }
    
    //x is the same unit as the wheel radius defined above (to be defined in robotMap later)
    public void travel(int x) {

        //start timer for dt
        lastTimestamp = Timer.getFPGATimestamp();

        //runs while not at target yet
        while(!finishedTravel()){
            setMotors(getMotorOutput(x));
        }

        //stops robot
        setMotors(0);
    }

    //PID calculation
    private double getMotorOutput(int x) {

        //get lastError
        double lastError = error;

        //calculate new error (e(t))
        error = x - encoder0.getDistance();

        //calculate P
        double P = kP * error;

        //calculate dt
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        lastTimestamp = Timer.getFPGATimestamp();

        //calculate errorSum (integral of e(t))
        errorSum += error * dt;

        //calculate I
        double I = kI * errorSum;

        //calculate errorRate (derivative of e(t))
        errorRate = (lastError - error) / dt;

        //calculate D
        double D = kD * errorRate;

        return P + I + D;

    }

    //distance robot can be from target to stop
    double errorDistance = 0.5;

    //detects if close enough to target to stop
    private boolean finishedTravel() {

        if(Math.abs(error) <= errorDistance ){
            return true;
        }

        return false;

    }

    /*
    private void calcError(double distance, double traveled) {
        error = distance - traveled; 
    }

    private double motorOutputPercentage(int distanceToTravel) {
        calcError(distanceToTravel, encoder0.getDistance());
        return kP * error;
    }
    private boolean finishedTravel() {
        calcError(error, kP);
        if(Math.abs(error) <= 0.5){
                return true;
            }
        return false;
    }
    */

    public void drive() {
        if ((IO.getDriveY()) > 1 || (IO.getDriveY()) < -1) {
            System.out.println("out of bounds drive value. go to Drivetrain.java line ?? and edit to an in-bounds expression");
        } else {
            // drive.arcadeDrive(IO.getThrottle() * DRIVE_SPEED_MULT, IO.getLeftXAxis() * DRIVE_SPEED_MULT);
            if(IO.getDriveY() < 0){

                diffDrive0.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT);
                diffDrive1.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT);
               // IO.putNumberToSmartDashboard(("Vel. R"), frontRight.getSelectedSensorVelocity());
               // IO.putNumberToSmartDashboard(("Vel. L"), frontLeft.getSelectedSensorVelocity());
                
                // System.out.println("Throttle: " + (Math.pow(IO.getThrottle(), 2) / 10));

            } else {
                diffDrive0.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT, true);
                diffDrive1.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT, true);
            }
           // IO.putNumberToSmartDashboard(("R Enc"),  frontLeft.getSelectedSensorPosition());
           // IO.putNumberToSmartDashboard(("L Enc"),  frontRight.getSelectedSensorPosition());
         //   IO.putNumberToSmartDashboard(("Average Drive Enc Value"),  IO.getDriveDistance(frontRight.getSelectedSensorPosition(), frontLeft.getSelectedSensorPosition(), true));
        }
    }
}

