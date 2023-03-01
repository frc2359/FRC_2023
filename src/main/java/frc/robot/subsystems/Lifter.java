package frc.robot.subsystems;
//Import SPARK MAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;


public class Lifter {
        
    //Create SPARK MAX object
    private CANSparkMax spark = new CANSparkMax(1, MotorType.kBrushless);
    private RelativeEncoder e = spark.getEncoder();
    private DigitalInput dio = new DigitalInput(2);

    //Set PID constants
    double kP = 0.1;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    public void init() {

    }

    public void run(double setpoint) {
        //Set feedback device as integrated encoder
        spark.getPIDController().setFeedbackDevice(spark.getEncoder());

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
        SmartDashboard.putNumber("Lifter Encoder", e.getPosition());
        if(dio.get()) {
            e.setPosition(0);
            spark.set(0);
        } else {
            if(IO.getPOV() == -1) {
                spark.set(0);
            } else if (IO.getPOV() == 180) {
                spark.set(-0.5);
            } else if (IO.getPOV() == 0) {
                spark.set(0.5);
            } else {
                spark.set(0);
            }
        }
    }
}
