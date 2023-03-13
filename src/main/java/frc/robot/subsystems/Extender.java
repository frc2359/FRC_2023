package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//Import SPARK MAX libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.ExtenderConstants.*;

public class Extender {
    private static TalonSRX extendMot;
    private int state = 0;
    private double distance = 0;
    private boolean autoCompleted = false;

    public void init() {
        extendMot = new TalonSRX(EXTEND_MOT_ID);
        // gripMot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        extendMot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        //0 encoder
    }

    /**Control via external systems */
    public void switchState(int st) {
        this.state = st;
    }

    public boolean extendToDistance(double dist) {
        
        this.distance = dist;
        this.state = CASE_EXTEND_TO_DIST;
        extendEm();
        return autoCompleted;
    }

    public double getDistanceInches() {
        return extendMot.getSelectedSensorPosition() / 195.44;
    }

    public double getRawDistance(double dist) {
        return dist * 195.44;
    }

    /**Case system that directly controls movement */
    public void extendEm() {
        SmartDashboard.putNumber("Extender Encoder", extendMot.getSelectedSensorPosition());
        SmartDashboard.putNumber("Extender Inches", getDistanceInches());
        SmartDashboard.putNumber("Extender Case", state);
        SmartDashboard.putBoolean("Extender Forward LS", extendMot.getSensorCollection().isFwdLimitSwitchClosed());
        SmartDashboard.putBoolean("Extender Reverse LS", extendMot.getSensorCollection().isRevLimitSwitchClosed());

        

        switch(state) {
            case CASE_ZERO_ENCODERS:
                extendMot.setSelectedSensorPosition(0);
                state = CASE_STOP;
            case CASE_STOP:
                extendMot.set(ControlMode.PercentOutput, 0);
                break;
            case CASE_UP:
                extendMot.set(ControlMode.PercentOutput, .75 * IO.getLiftControlRightX());
                
                break;
            case CASE_UP_SLOW:
                extendMot.set(ControlMode.PercentOutput, .4 * IO.getLiftControlRightX());
                break;
            case CASE_DOWN:
                extendMot.set(ControlMode.PercentOutput, .75 * IO.getLiftControlRightX());
                
                break;
            case CASE_DOWN_SLOW:
                extendMot.set(ControlMode.PercentOutput, .4 * IO.getLiftControlRightX());
                break;
            case CASE_EXTEND_TO_DIST:
                if(extendMot.getSelectedSensorPosition() >= distance) {
                    extendMot.set(ControlMode.PercentOutput, 0);
                    autoCompleted = true;
                } else {
                    extendMot.set(ControlMode.PercentOutput, .3);
                }
                break;
        }
    }

    /**Runs the system manually. Control via joystick */
    public void manualRun() {
        extendEm();

        if(SEPARATE_CONTROLS ? IO.getLiftControlRightX() > 0.1 : IO.isPOVToAngle(90)){
            extendMot.set(ControlMode.PercentOutput, 0.5);
            if(getDistanceInches() < 5) {
                state = CASE_UP_SLOW;
            } else {
                state = CASE_UP;
            }
        }

        if(SEPARATE_CONTROLS ? IO.getLiftControlRightX() < -0.1 : IO.isPOVToAngle(270)){
            if(extendMot.getSensorCollection().isRevLimitSwitchClosed()) {
                extendMot.set(ControlMode.PercentOutput, 0);
                state = CASE_ZERO_ENCODERS;
            } else {
                if(getDistanceInches() < 5) {
                    state = CASE_DOWN_SLOW;
                } else {
                    state = CASE_DOWN;
                }
            }
        } else {
            state = CASE_STOP;
        }

        
    }

    
}
