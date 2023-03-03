package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.RobotMap.*;
import frc.robot.IO;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Gripper {
    

    private static TalonSRX gripMot = new TalonSRX(69); // set # later
    private DigitalInput openLim = new DigitalInput(ClawConstants.OPEN_LIM);
    private DigitalInput closeLim = new DigitalInput(ClawConstants.CLOSE_LIM);

    public void gripEm(){
        if(!openLim.get()){
            SmartDashboard.putBoolean("Is it open? :)", openLim.get());
            gripMot.set(ControlMode.PercentOutput, 0);
        } 

        if(!closeLim.get()){
            SmartDashboard.putBoolean("Is it closed? :(", closeLim.get());
            gripMot.set(ControlMode.PercentOutput, 0);
        }

        if(IO.getButton(10)){
            gripMot.set(ControlMode.PercentOutput, 0.5);
        }

        if(IO.getButton(7)){
            gripMot.set(ControlMode.PercentOutput, -0.5);
        }

        for(int j = 0; j < 12; j++){
            System.out.println(j);
        }


    }

    // private boolean activated = false;
    // private static WPI_VictorSPX gripDevice = new WPI_VictorSPX(8);
    // private static Joystick buttons = new Joystick(0);
    // private static DigitalInput limSwitch = new DigitalInput(7);

    // public void gripEm(){
        
    //     if(buttons.getRawButtonPressed(8)){
    //         activated = true;
    //         while(limSwitch.get() == false){
    //         gripDevice.set(.6);
    //         }
    //     } 
    //     if(buttons.getRawButtonPressed(10)) {
    //         activated = true;
    //         gripDevice.set(.6);
    //     } 
    //     if(buttons.getRawButtonPressed(12)){
    //         activated = false;
    //     }
    //     if(!activated){
    //         gripDevice.set(0);
    //     }
    // }
    

}

