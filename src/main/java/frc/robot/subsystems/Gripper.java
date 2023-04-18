package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.ClawConstants.*;

public class Gripper {
    
    private static VictorSPX gripMotorLeft;
    private static VictorSPX gripMotorRight;

    private final ControlMode PO = ControlMode.PercentOutput;
    //dio 9
    private DigitalInput coneDio = new DigitalInput(DIO_HAS_CONE);
    private DigitalInput cubeDio = new DigitalInput(DIO_HAS_CUBE);


    public int state = 0;
    public int count = 0;
    private double setpoint = 0;


    /**Initialize the gripper */
    public void init() {
        gripMotorLeft = new VictorSPX(CLAW_LEFT_MOT_ID);
        gripMotorRight = new VictorSPX(CLAW_RIGHT_MOT_ID);

        gripMotorLeft.follow(gripMotorRight);
        gripMotorRight.setInverted(InvertType.OpposeMaster);
    }

    /**Returns true + rumble if an cone is in the gripper */
    public boolean hasCone() {
        if(!coneDio.get()) {
            IO.setRumble(1, kLeft);
        } else {
            IO.setRumble(0, kBoth);
        }
        SmartDashboard.putBoolean("HasCone?", !coneDio.get());
        return !coneDio.get();
    }

    public boolean hasCube() {
        if(!coneDio.get()) {
            IO.setRumble(1, kRight);
        } else {
            IO.setRumble(0, kBoth);
        }
        SmartDashboard.putBoolean("HasCube?", !cubeDio.get());
        return !cubeDio.get();
    }

    /**Sets the gripper state */
    public Gripper setState(int st) {
        this.state = st;
        return this;
    }

    /** Run the gripper. */
    public void run() {
        switch(state) {
            case CASE_WAIT:
                count++;
                if(count == 50) {
                    count = 0;
                    state = CASE_STOP;
                }
                break;
            case CASE_STOP:
                setpoint = 0;
                break;
            case CASE_INTAKE:
                setpoint = 0.8;
                break;
            case CASE_POWERED_HOLD:
                setpoint = 0.3;
                break;
            case CASE_EXPEL_CUBE_HIGH:
                setpoint = -1;
                break;
            case CASE_EXPEL_CUBE_MID:
                setpoint = -0.8;
                break;
            case CASE_EXPEL_CUBE_LOW:
                setpoint = -0.5;
                break;
            case CASE_EXPEL_CONE:
                setpoint = -0.6;
                break;
        }
        gripMotorRight.set(PO, setpoint);
    }

    /**Manual Control of the gripper. */
    public void manualControl() {
        hasCone();
        if (IO.isAPressed() || IO.getHIDButton(CMD_BUTTON_INTAKE)) {  this.state = CASE_INTAKE;  } //intake
        
        else if(IO.getLiftPOV() == 0 || IO.getHIDButton(CMD_BUTTON_EXP_FA)) { this.state = CASE_EXPEL_CUBE_HIGH; }
        else if(IO.getLiftPOV() == 180 || IO.getHIDButton(CMD_BUTTON_EXP_SL)) { this.state = CASE_EXPEL_CUBE_LOW; }
        else if (IO.isBPressed() || IO.getHIDButton(CMD_BUTTON_STOP)) {  this.state = CASE_STOP;  } // stop motor

        if (this.state == CASE_INTAKE && hasCone()) {
            this.state = CASE_WAIT;
        }

        run();
    }
}
