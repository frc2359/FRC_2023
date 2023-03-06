package frc.robot.subsystems;
//Import SPARK MAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.LifterConstants.*;


public class Lifter {
        
    //Create SPARK MAX object
    private CANSparkMax spark = new CANSparkMax(LIFT_ID, MotorType.kBrushless);
    private SparkMaxPIDController s_Pid;
    private RelativeEncoder e = spark.getEncoder();
    private DigitalInput dio = new DigitalInput(LIFT_LIMIT);

    //Set PID constants
    double kP = 0.1;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    public void init() {
        // e.setPositionConversionFactor((1/233.33));
        s_Pid = spark.getPIDController();
        // s_Pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);

        s_Pid.setP(kP);
        s_Pid.setI(kI);
        s_Pid.setD(kD);
        s_Pid.setIZone(kIz);
        s_Pid.setFF(kFF);
        s_Pid.setOutputRange(kMinOutput, kMaxOutput);

        spark.getPIDController().setFeedbackDevice(spark.getEncoder());
    }

    public double getRotationAngle() {
        return ((e.getPosition() / 233.33) * 360);
    }

    public void run(double setpoint) {
        //Set feedback device as integrated encoder
        // spark.getPIDController().setFeedbackDevice(spark.getEncoder());

        //Set PID coefficients
        spark.getPIDController().setP(kP);
        spark.getPIDController().setI(kI);
        spark.getPIDController().setD(kD);
        spark.getPIDController().setIZone(kIz);
        spark.getPIDController().setFF(kFF);
        spark.getPIDController().setOutputRange(kMinOutput, kMaxOutput);

        //Send desired position to controller
        spark.getPIDController().setReference(setpoint, ControlType.kPosition);
    }

    public void manualRun() {
        SmartDashboard.putNumber("Lifter Encoder", ((e.getPosition() / 233.33) * 360));
        SmartDashboard.putNumber("Lifter Conv. Fact.", e.getPositionConversionFactor());
        SmartDashboard.putBoolean("DIO", dio.get());
        SmartDashboard.putNumber("POV", IO.getPOV());


        if(!dio.get()) {
            e.setPosition(0);
            spark.set(0);
        } else {
            if(IO.getPOV() == -1) {
                s_Pid.setReference(0, ControlType.kVoltage);
            } else if (IO.getPOV() == 180) {
                spark.set(-0.75);
                // s_Pid.setReference(-0.75, ControlType.kVoltage);
            } else if (IO.getPOV() == 0) {
                spark.set(0.75);
                // s_Pid.setReference(0.75, ControlType.kVoltage);
            } else {
                s_Pid.setReference(0, ControlType.kVoltage);
            }
        }
    }
}
