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
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.ExtenderConstants.*;

public class Extender {
    private static TalonSRX gripMot;
    private int state = 0;
    private double distance = 0;

    public void init() {
        gripMot = new TalonSRX(EXTEND_MOT_ID);
        // gripMot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        gripMot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        //0 encoder
    }

    /**Control via external systems */
    public void switchState(int st) {
        this.state = st;
    }

    public void extendToDistance(double dist) {
        this.distance = dist;
        this.state = CASE_EXTEND_TO_DIST;
    }

    /**Case system that directly controls movement */
    public void extendEm() {
        SmartDashboard.putNumber("Extender Encoder", gripMot.getSelectedSensorPosition());

        switch(state) {
            case CASE_ZERO_ENCODERS:
                gripMot.setSelectedSensorPosition(0);
                state = CASE_STOP;
            case CASE_STOP:
                gripMot.set(ControlMode.PercentOutput, 0);
                break;
            case CASE_UP:
                gripMot.set(ControlMode.PercentOutput, 0.5);
                break;
            case CASE_DOWN:
                gripMot.set(ControlMode.PercentOutput, -0.5);
                break;
            case CASE_EXTEND_TO_DIST:
                gripMot.set(ControlMode.Position, distance);
                break;
        }
    }

    /**Runs the system manually. Control via joystick */
    public void manualRun() {
        extendEm();

        if(IO.getButton(10) || (!IO.getButton(3) && !IO.getButton(4))){
            state = CASE_STOP;
        }

        if(IO.getButton(3)){
            gripMot.set(ControlMode.PercentOutput, 0.5);
            state = CASE_UP;
        }

        if(IO.getButton(4)){
            gripMot.set(ControlMode.PercentOutput, -0.5);
            state = CASE_DOWN;
        }

        if(state == -1 && gripMot.getSensorCollection().isRevLimitSwitchClosed()) {
            state = CASE_ZERO_ENCODERS;
        }

        if(state == 1 && gripMot.getSensorCollection().isFwdLimitSwitchClosed()) {
            state = CASE_ZERO_ENCODERS;
        }
    }
}
