package frc.robot.subsystems;

//Import SPARK MAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.LifterConstants.*;

public class Lifter {

    // Create SPARK MAX object
    private CANSparkMax spark = new CANSparkMax(LIFT_ID, MotorType.kBrushless);
    private SparkMaxPIDController s_Pid;
    private RelativeEncoder e = spark.getEncoder();
    private DigitalInput dio = new DigitalInput(LIFT_LIMIT);

    private double setpoint;
    private double spdLifter;
    private int state = STATE_LIFT_UNKOWN;

    // Set PID constants
    double kP = 0.0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    public void init() {
        e.setPositionConversionFactor((1 / (5 * 5 * 4 * 4)));
        s_Pid = spark.getPIDController();

        spark.setOpenLoopRampRate(kRampRateSec);

        s_Pid.setSmartMotionMaxAccel(kMaxAngularAccelerationRadiansPerSecondSquared, LIFT_ID);
        s_Pid.setSmartMotionMaxVelocity(kMaxAngularSpeedRadiansPerSecond, LIFT_ID);
        s_Pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, LIFT_ID);

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

    public void setToDistance(double setpoint) {
        setpoint = setpoint < 0.0 ? 0 : setpoint;
        setpoint = setpoint > LIFTER_MAX_ROTATION ? LIFTER_MAX_ROTATION : setpoint;

        this.setpoint = setpoint;

        if (state != STATE_LIFT_UNKOWN) {
            state = STATE_LIFT__MOVE_TO_POS;
        }
    }

    public void run() {
        SmartDashboard.putNumber("Lifter Encoder", (e.getPosition()));
        SmartDashboard.putNumber("Lifter Conv. Fact.", e.getPositionConversionFactor());
        SmartDashboard.putBoolean("DIO3", !dio.get());

        switch (state) {
            case STATE_LIFT_UNKOWN:
                spdLifter = -0.1;
                if (!dio.get()) {
                    state = STATE_LIFT_ZERO_ENCODERS;
                }
                break;
            case STATE_LIFT_ZERO_ENCODERS:
                e.setPosition(0);
                spdLifter = 0;
                state = STATE_LIFT_STOP;
            case STATE_LIFT_STOP:
                spdLifter = 0;
                break;
            case STATE_LIFT_UP:
                spdLifter = Math.abs(spdLifter);
                break;
            case STATE_LIFT_DOWN:
                spdLifter = -Math.abs(spdLifter);
                break;
            case STATE_LIFT__MOVE_TO_POS:
                spark.getPIDController().setReference(this.setpoint, ControlType.kPosition);
                break;

        }
        spark.set(spdLifter * kMaxMultiplier);
        // Send desired position to controller
    }

    public boolean autoRun(double rot) {
        spdLifter = 1;
        setToDistance(rot);
        state = STATE_LIFT__MOVE_TO_POS;
        run();
        return true;
    }

    public void manualRun() {
        spdLifter = IO.getLiftControlLeftY();

        if (IO.getLiftControlLeftY() > 0.5) {
            state = STATE_LIFT_UP;
        } else if (IO.getLiftControlLeftY() < -0.5) {
            state = STATE_LIFT_DOWN;
        }

        run();
    }
}
