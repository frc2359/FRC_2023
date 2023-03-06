package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.ClawConstants.*;
import frc.robot.IO;


public class Gripper {
    

    private static TalonSRX gripMotor = new TalonSRX(CLAW_MOT_ID);
    private int state = 0;


    public void init() {
        gripMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        gripMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    public void switchState(int st) {
        this.state = st;
    }

    public void gripEm() {
        switch(state){
            case CASE_STOP:
                gripMotor.set(ControlMode.PercentOutput, 0);
            break;
            case CASE_OPEN:
                gripMotor.set(ControlMode.PercentOutput, 0.4);
                break;
            case CASE_CLOSED:
                gripMotor.set(ControlMode.PercentOutput, -0.4);
                break;
        }
    }

    public void manualRun() {
        gripEm();

        if(state == CASE_OPEN) {
            SmartDashboard.putBoolean("Gripper Opened", true);
        } else if (state == CASE_CLOSED) {
            SmartDashboard.putBoolean("Gripper Opened", false);
        }

        if(IO.getButton(10)){
            state = CASE_CLOSED;
        }

        if(IO.getButton(3)){
            state = CASE_OPEN;
        }

        if(IO.getButton(4)){
            state = CASE_CLOSED;
        }

        if(state == CASE_CLOSED && gripMotor.getSensorCollection().isRevLimitSwitchClosed()) {
            gripMotor.set(ControlMode.PercentOutput, 0);
        }

        if(state == CASE_OPEN && gripMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
            gripMotor.set(ControlMode.PercentOutput, 0);
        }

    }
}

