package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.IO;
import frc.robot.IO.*;
import static frc.robot.RobotMap.*;

public class SimpleDrive {
    public final VictorSP drive0 = new VictorSP(0);
    public final VictorSP drive1 = new VictorSP(1);
    public final VictorSP drive2 = new VictorSP(2);
    public final VictorSP drive3 = new VictorSP(3);
    public final Joystick joy = new Joystick(0);
    public final Encoder encoder0 = new Encoder(0, 1);

    private DifferentialDrive diffDrive0 = new DifferentialDrive(drive0, drive1);
    private DifferentialDrive diffDrive1 = new DifferentialDrive(drive2, drive3);


    private double error = 0;
    private double kP = 0.000001;
    private double kI = 0.000001;
    private double kD = 0.000001;
    private double errorsum = 0;
    private double lastTimestamp = 0;

    public void init() {
        drive0.setSafetyEnabled(true);
        drive1.setSafetyEnabled(true);
        drive2.setSafetyEnabled(true);
        drive3.setSafetyEnabled(true);

        // drive0.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        // drive1.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        // drive2.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
        // drive3.setNeutralMode(BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setPIDValues(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
    
    public void travel(int x) {
        while(!finishedTravel()){
            drive0.set(motorOutputPercentage(x));
            drive1.set(motorOutputPercentage(x));
            drive2.set(motorOutputPercentage(x));
            drive3.set(motorOutputPercentage(x));
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

