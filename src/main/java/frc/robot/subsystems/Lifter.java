package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private int stateLifter = STATE_LIFT_UNKOWN;

    private boolean homed = false; //to know if the lifter is homed
    private int count = 0; // this allows us to save the initial position when the control is stopped so that we can hold it
    private double holdPos;

    private double deadband;

    // Set PID constants
    double kP = 0.0;
    double kI = 0.0;
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

        homed = false;
        count = 0;
    }

    public void showInfo() {
        SmartDashboard.putNumber("Lifter Encoder", (e.getPosition()));
        SmartDashboard.putNumber("Lifter Case", stateLifter);
        SmartDashboard.putBoolean("DIO3", !dio.get());
        SmartDashboard.putNumber("Ext Max Rot Calc", getMaxRotation());
    }

    public void setStateLifter(int stateSetpoint){
        this.stateLifter = stateSetpoint;
    }

    public int getStateLifter() {
        return this.stateLifter;
    }

    public double getRotationAngle() {
        return ((e.getPosition() / 233.33) * 360);
    }

    public double getMaxRotation() {
        return LIFTER_MAX_ROTATION;
        // return 180 - Math.toDegrees(Math.acos(14 / (16.5 + (ExtenderConstants.currentlyExtended))));
    }

    public void setToDistance(double setpoint) {
        setpoint = setpoint < 0.0 ? 0 : setpoint;
        setpoint = setpoint > getMaxRotation() ? getMaxRotation() : setpoint;

        this.setpoint = setpoint;

        if (stateLifter != STATE_LIFT_UNKOWN) {
            stateLifter = STATE_LIFT__MOVE_TO_POS;
        }
    }

    public void setToDistance(double setpoint, double deadband) {
        setpoint = setpoint < 0.0 ? 0 : setpoint;
        setpoint = setpoint > getMaxRotation() ? getMaxRotation() : setpoint;

        this.setpoint = setpoint;
        this.deadband = deadband;

        if (stateLifter != STATE_LIFT_UNKOWN) {
            stateLifter = STATE_LIFT__MOVE_TO_POS;
        }
    }

    public void run() {
        SmartDashboard.putNumber("Lifter Encoder", (e.getPosition()));
        SmartDashboard.putNumber("Lifter Case", stateLifter);
        SmartDashboard.putBoolean("DIO3", !dio.get());
        SmartDashboard.putNumber("Ext Max Rot Calc", getMaxRotation());

        switch (stateLifter) {
            case STATE_LIFT_UNKOWN:
                spdLifter = -0.09;
                if (!dio.get()) {
                    stateLifter = STATE_LIFT_ZERO_ENCODERS;
                }
                break;
            case STATE_LIFT_ZERO_ENCODERS:
                this.homed = true;
                e.setPosition(0);
                spdLifter = 0;
                stateLifter = STATE_LIFT_STOP;
            case STATE_LIFT_STOP:
                spdLifter = 0;

                // spdLifter = 0.1 * (setpoint - e.getPosition());

                break;
            case STATE_LIFT_UP:
                spdLifter = Math.abs(spdLifter);
                if(e.getPosition() <= 5 || e.getPosition() >= getMaxRotation()) {
                    stateLifter = STATE_LIFT_STOP;
                }
                setpoint = e.getPosition();
                break;
            case STATE_LIFT_DOWN:
                spdLifter = -Math.abs(spdLifter);
                if(e.getPosition() <= 0 || e.getPosition() >= getMaxRotation()) {
                    stateLifter = STATE_LIFT_STOP;
                }
                setpoint = e.getPosition();
                break;
            case STATE_LIFT__MOVE_TO_POS:
                // spark.getPIDController().setReference(this.setpoint, ControlType.kPosition);
                if (e.getPosition() >= (setpoint + (1.5 * deadband))){
                    spdLifter = -1;
                } else if (e.getPosition() >= (setpoint + deadband)){
                    spdLifter = -0.4;
                } else if (e.getPosition() <= (setpoint - (1.5 * deadband))){
                    spdLifter = 1;
                } else if (e.getPosition() <= (setpoint - (deadband))){
                    spdLifter = 0.4;
                } else {
                    spdLifter = 0;
                }

                break;

        }
        spark.set(spdLifter * kMaxMultiplier);
        // Send desired position to controller
    }

    /**Runs the lifter to a specified rotation
     * @param rot is the rotation setpoint
     * @param deadband is the acceptable +/- limit
     */
    public boolean autoRun(double rot, double deadband) { // this.setpoint = rot, this.deadband = deadband

        this.setpoint = rot;
        this.deadband = deadband;

        if (homed) {
            setToDistance(rot);
            spdLifter = 0.7;
            stateLifter = STATE_LIFT__MOVE_TO_POS;
        }
        run();
        return (e.getPosition() >= rot - deadband && e.getPosition() <= rot + deadband);
    }

    /**return true if the lifter has been homed */
    public boolean isHomed() { 
        return homed;
    }

    public void setDriveMode(boolean isBrake) {
        spark.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        SmartDashboard.putBoolean("LiftBrakeMode", spark.getIdleMode() == IdleMode.kBrake);
    }

    public boolean currentlyHomed() {
        return !dio.get();
    }

    public void manualRun() {
        spdLifter = IO.getLiftControlLeftY();

        if (IO.getLiftControlLeftY() > 0.5 && e.getPosition() < getMaxRotation()) {
            SmartDashboard.putBoolean("Holding", false);
            stateLifter = STATE_LIFT_UP;
            count = 0;
        } else if (IO.getLiftControlLeftY() < -0.5 && e.getPosition() > 5) {
            SmartDashboard.putBoolean("Holding", false);
            stateLifter = STATE_LIFT_DOWN;
            count = 0;
        } else if (homed && IO.getLiftControlLeftY() > -0.5 && IO.getLiftControlLeftY() < 0.5) {
            SmartDashboard.putNumber("Hold Position", holdPos);
            SmartDashboard.putBoolean("Holding", true);
            if (count == 0) {
                holdPos = e.getPosition();
            }
            spark.getPIDController().setReference(holdPos, ControlType.kPosition);
            count++;
        }

        
        run();
    }

}
