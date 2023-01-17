package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.IO;
import frc.robot.IO.*;
import static frc.robot.RobotMap.*;

public class SimpleDrive {
    public final WPI_VictorSPX drive1 = new WPI_VictorSPX(1);
    public final WPI_VictorSPX drive2 = new WPI_VictorSPX(2);
    public final WPI_VictorSPX drive3 = new WPI_VictorSPX(3);
    public final WPI_VictorSPX drive4 = new WPI_VictorSPX(4);
    public final Joystick joy = new Joystick(0);

    //encoder
    public final Encoder encoder0 = new Encoder(0, 1);

    private DifferentialDrive diffDrive = new DifferentialDrive(drive1, drive2);
    // private DifferentialDrive diffDrive1 = new DifferentialDrive(drive3, drive4);

    //PID UNKNOWNS
    private double error = 0;
    private double errorSum = 0;
    private double errorRate = 0;

    //PID KNOWNS (HARDCODE AFTER TRIAL AND ERROR)
    //THESE VALUES WILL CHANGE IF THE DIMENSIONS OF THE ROBOT CHANGE
    //THESE ARE DEFAULTS - CHANGE
    private double kP = 0.0001;
    private double kI = 0.0001;
    private double kD = 0.0001;

    //TIMER
    private double lastTimestamp = 0;

    //initialize safety and follow for wheels
    public void init() {
        drive1.setSafetyEnabled(true);
        drive2.setSafetyEnabled(true);
        drive3.setSafetyEnabled(true);
        drive4.setSafetyEnabled(true);

        drive1.setInverted(true);
        // drive3.setInverted(true);
        drive4.setInverted(true);
        // drive2.setInverted(true);

        drive3.follow(drive2);
        drive4.follow(drive1);

        drive1.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        drive2.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        drive3.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        drive4.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /** set PID values for testing 
     * @param p is the proportional value
     * @param i is the integral value
     * @param d is the derivative value
    */
    public void setPIDValues(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    /**  sets all motor outputs to same value */
    public void setMotors(double val) {
        drive1.set(val);
        drive2.set(val);
        drive3.set(val);
        drive4.set(val);
    }
    
    /*
    //x is the same unit as the wheel radius defined above (to be defined in robotMap later)
    public void travel(int x) {

        //start timer for dt
        lastTimestamp = Timer.getFPGATimestamp();

        //runs while not at target yet
        while(!finishedTravel()){
            drive0.set(motorOutputPercentage(x));
            drive1.set(motorOutputPercentage(x));
            drive2.set(motorOutputPercentage(x));
            drive3.set(motorOutputPercentage(x));
        }

        return false;

    }

    
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

    private void feed() {
        drive1.feed();
        drive2.feed();
        drive3.feed();
        drive4.feed();
    }

    /**Simple arcade drive mechanism */
    public void drive() {
        feed();
        if ((IO.getDriveY()) > 1 || (IO.getDriveY()) < -1) {
            System.out.println("out of bounds drive value. go to Drivetrain.java line ?? and edit to an in-bounds expression");
        } else {
            diffDrive.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT);
            // diffDrive1.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT);
        }
    }
}

