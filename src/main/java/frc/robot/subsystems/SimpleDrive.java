package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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
    public final Encoder encoder0 = new Encoder(0, 1);

    private DifferentialDrive diffDrive = new DifferentialDrive(drive1, drive2);
    private DifferentialDrive diffDrive1 = new DifferentialDrive(drive3, drive4);


    private double error = 0;
    private double kP = 0.000001;
    private double kI = 0.000001;
    private double kD = 0.000001;
    private double errorsum = 0;
    private double lastTimestamp = 0;

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

    public void setPIDValues(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
    
    public void travel(int x) {
        while(!finishedTravel()){
            drive1.set(motorOutputPercentage(x));
            drive2.set(motorOutputPercentage(x));
            drive3.set(motorOutputPercentage(x));
            drive4.set(motorOutputPercentage(x));
        }
    }

    private void calcError(double traveled, double distance) {
        error = distance - traveled; 
    }

    private double motorOutputPercentage(int distanceToTravel) {
        calcError(distanceToTravel, encoder0.getDistance());
        return kP * error;
    }
    private boolean finishedTravel() {
        calcError(kP, error);
        if(Math.abs(error) <= 0.5){
                return true;
            }
        return false;
    }

    public void drive() {
        if ((IO.getDriveY()) > 1 || (IO.getDriveY()) < -1) {
            System.out.println("out of bounds drive value. go to Drivetrain.java line 34 and edit to an in-bounds expression");
        } else {
            // drive.arcadeDrive(IO.getThrottle() * DRIVE_SPEED_MULT, IO.getLeftXAxis() * DRIVE_SPEED_MULT);
            if(IO.getDriveY() < 0){

                diffDrive.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT);
                diffDrive1.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT);
               // IO.putNumberToSmartDashboard(("Vel. R"), frontRight.getSelectedSensorVelocity());
               // IO.putNumberToSmartDashboard(("Vel. L"), frontLeft.getSelectedSensorVelocity());
                
                // System.out.println("Throttle: " + (Math.pow(IO.getThrottle(), 2) / 10));

            } else {
                diffDrive.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT, true);
                diffDrive1.arcadeDrive(IO.getDriveY(), IO.getDriveX() * TURN_SPEED_MULT, true);
            }
           // IO.putNumberToSmartDashboard(("R Enc"),  frontLeft.getSelectedSensorPosition());
           // IO.putNumberToSmartDashboard(("L Enc"),  frontRight.getSelectedSensorPosition());
         //   IO.putNumberToSmartDashboard(("Average Drive Enc Value"),  IO.getDriveDistance(frontRight.getSelectedSensorPosition(), frontLeft.getSelectedSensorPosition(), true));
        }
    }
}

