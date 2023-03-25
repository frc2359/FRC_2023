package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.ClawConstants.*;

public class Gripper {
    
    private static VictorSPX gripMotorLeft;
    private static VictorSPX gripMotorRight;

    private final ControlMode PO = ControlMode.PercentOutput;
    //dio 9
    private DigitalInput dio = new DigitalInput(9);


    public int state = 0;
    private double setpoint = 0;


    /**Initialize the gripper */
    public void init() {
        gripMotorLeft = new VictorSPX(CLAW_LEFT_MOT_ID);
        gripMotorRight = new VictorSPX(CLAW_RIGHT_MOT_ID);

        gripMotorLeft.follow(gripMotorRight);
        gripMotorRight.setInverted(InvertType.OpposeMaster);
    }

    /**<b>NEEDS INITIALIZATION</b> -- Returns true if an object is in the gripper */
    public boolean lineTripped() {
        //rumble
        SmartDashboard.putBoolean("Gripper Line", !dio.get());
        return !dio.get();
    }

    public void setState(int st) {
        this.state = st;
    }

    public void run() {
        switch(state) {
            case CASE_STOP:
                setpoint = 0;
                break;
            case CASE_INTAKE:
                setpoint = 0.6;
                break;
            case CASE_POWERED_HOLD:
                setpoint = 0.1;
                break;
            case CASE_EXPEL_CUBE_HIGH:
                setpoint = -1;
                break;
            case CASE_EXPEL_CUBE_MID:
                setpoint = -0.5;
                break;
            case CASE_EXPEL_CUBE_LOW:
                setpoint = -0.3;
                break;
            case CASE_EXPEL_CONE:
                setpoint = -0.4;
                break;
        }
        gripMotorRight.set(PO, setpoint);
    }

    /**Manual Control of the gripper.
     * <ul>
     * <li><b>A</b>: Intake
     * <li><b>X</b>: Expel Cube Selector
     * <ul>
     * <li><b>Y</b>: High
     * <li><b>B</b>: Middle
     * <li><b>A</b>: Low
     * </ul>
     * <li><b>Y</b>: Expel Cone
     * </ul>
     */
    public void manualControl() {
        lineTripped();
        if (!IO.isXPressed() && IO.isAPressed()) {  this.state = CASE_INTAKE;  } //intake
        
        else if (IO.isXPressed()) { //cube expel selector
            if(IO.isXPressed() && IO.isYPressed()) { this.state = CASE_EXPEL_CUBE_HIGH; }
            else if(IO.isXPressed() && IO.isBPressed()) { this.state = CASE_EXPEL_CUBE_MID; }
            else if(IO.isXPressed() && IO.isAPressed()) { this.state = CASE_EXPEL_CUBE_LOW; }
        } else if (!IO.isXPressed() && IO.isYPressed()) {  this.state = CASE_EXPEL_CONE;  } //expel cone
          else if (!IO.isXPressed() && IO.isBPressed()) {  this.state = CASE_STOP;  } // stop motor

        if (this.state == CASE_INTAKE && lineTripped()) {
            this.state = CASE_POWERED_HOLD;
        }

        run();
    }


    //expel cube

    //expelMid cube

    //expelHigh cube

    //expel Cone

}
