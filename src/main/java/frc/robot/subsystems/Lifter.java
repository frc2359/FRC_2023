package frc.robot.subsystems;
//Import SPARK MAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    double kP = 0.2;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private static int states = 0; 

    public void init() {
        e.setPositionConversionFactor((1/(5*5*4*4)));
        s_Pid = spark.getPIDController();
        s_Pid.setSmartMotionMaxAccel(kMaxAngularAccelerationRadiansPerSecondSquared, 0);
        s_Pid.setSmartMotionMaxVelocity(kMaxAngularSpeedRadiansPerSecond, 0);
        s_Pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

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

        //Send desired position to controller
        spark.getPIDController().setReference(setpoint, ControlType.kPosition);
    }

    public void autoRun(double rot) {
        SmartDashboard.putNumber("Lifter Encoder", (e.getPosition()));
        SmartDashboard.putNumber("Lifter Conv. Fact.", e.getPositionConversionFactor());
        SmartDashboard.putBoolean("DIO3", !dio.get());
        if(e.getPosition() >= rot) {
            spark.set(0);
        } else {
            spark.set(0.3);
        }

    }

    public void manualRun() {
        SmartDashboard.putNumber("Lifter Encoder", (e.getPosition()));
        SmartDashboard.putNumber("Lifter Conv. Fact.", e.getPositionConversionFactor());
        SmartDashboard.putBoolean("DIO3", !dio.get());

        TrapezoidProfile.State state = new TrapezoidProfile.State(7, 10);
        TrapezoidProfile trap = new TrapezoidProfile(kThetaControllerConstraints, state);

            if(SEPARATE_CONTROLS) {
                if(!dio.get()){
                    e.setPosition(0);
                }
                if( IO.getLiftControlLeftY() > 0.5) {
                    if(e.getPosition() >= 126){
                        spark.set(0);
                    } else{
                    // s_Pid.setReference(IO.getLiftControlLeftY() * kMaxVoltage, ControlType.kVoltage);
                        spark.set(.3 *  IO.getLiftControlLeftY());
                    }
                    
                } 
                else if( IO.getLiftControlLeftY() < -0.5){
                    if(dio.get()){
                    // s_Pid.setReference(IO.getLiftControlLeftY() * kMaxVoltage, ControlType.kVoltage);
                        spark.set(.3 *  IO.getLiftControlLeftY());
                    }
                }
                else{
                    spark.set(0);
                }
            }
                
            //     else {
            //         if(!dio.get() && IO.getLiftControlLeftY() < 0){
            //             spark.set(.5);
            //         }
            //         // s_Pid.setReference(IO.getLiftControlLeftY() * kMaxVoltage, ControlType.kVoltage);
                    

            //     }
            // }
        //  else if(!dio.get()) {
        //     e.setPosition(0);
        //     //spark.set(0);
        // } else {
        //     while(SEPARATE_CONTROLS ? IO.isXPressed() : IO.getButton(12)) {
        //         s_Pid.setReference(25, ControlType.kSmartMotion);
        //     }
        //     if(SEPARATE_CONTROLS) {
        //         if(e.getPosition() >= 126 && IO.getLiftControlLeftY() > 0) {
        //             spark.set(0);
        //         } else {

        //             s_Pid.setReference(IO.getLiftControlLeftY() * kMaxVoltage, ControlType.kVoltage);
        //         }
        //     }
        // }
            // } else {
            //     if(IO.isPOVToAngle(-1)) {
            //         s_Pid.setReference(0, ControlType.kVoltage);
            //     } else if (IO.isPOVToAngle(180)) {
            //         // spark.set(-0.75);
            //         s_Pid.setReference(-2, ControlType.kVoltage);
            //     } else if (IO.isPOVToAngle(0)) {
            //         s_Pid.setReference(2, ControlType.kVoltage);
            //         // spark.set(0.75);
            //         if(e.getPosition() >= 114) {
            //             spark.set(0);
            //         }
                   
            //     } else {
            //         s_Pid.setReference(0, ControlType.kVoltage);
            //     }

                
            // }

            
            
        }
    }

