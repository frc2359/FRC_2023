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


public class Gripper_OLD {
    

    private static TalonSRX gripMotor = new TalonSRX(CLAW_MOT_ID);
    private int state = 0;

    private final int CASE_OPEN = 1;    //  placed due to depreciation in RobotMap
    private final int CASE_CLOSED = 2;  //  placed due to depreciation in RobotMap



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
                gripMotor.set(ControlMode.PercentOutput, 0.6);
                break;
            case CASE_CLOSED:
                gripMotor.set(ControlMode.PercentOutput, -0.6);
                break;
        }
    }

    public void manualRun() {
        gripEm();
        //reverse = open
        SmartDashboard.putBoolean("Gripper Reverse", gripMotor.getSensorCollection().isRevLimitSwitchClosed());
        SmartDashboard.putBoolean("Gripper Forward", gripMotor.getSensorCollection().isFwdLimitSwitchClosed());

        if(state == CASE_OPEN) {
            SmartDashboard.putBoolean("Gripper Opened", true);
        } else if (state == CASE_CLOSED) {
            SmartDashboard.putBoolean("Gripper Opened", false);
        }

        if(OIConstants.SEPARATE_CONTROLS) {
            if(IO.isAPressed()) {
                state = CASE_CLOSED;
            } else if (IO.isYPressed()) {
                state = CASE_OPEN;
            } else if(IO.isBPressed()) {
                state = CASE_STOP;
            }
        } else {
            if(IO.getDriverButton(10)){
                state = CASE_STOP;
            }
    
            if(IO.getDriverButton(3)){
                state = CASE_OPEN;
            }
    
            if(IO.getDriverButton(4)){
                state = CASE_CLOSED;
            }
        }
    }
}

