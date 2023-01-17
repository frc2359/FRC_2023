package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.RobotMap.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;

public class Gripper {
    
    private boolean activated = false;
    private static WPI_VictorSPX gripDevice = new WPI_VictorSPX(8);
    private static Joystick buttons = new Joystick(0);
    private static DigitalInput limSwitch = new DigitalInput(0);


    public void gripEm(){
        
        if(buttons.getRawButtonPressed(8)){
            activated = true;
            while(limSwitch.get() == false){
            gripDevice.set(.6);
            }
        } 
        if(buttons.getRawButtonPressed(10)) {
            activated = true;
            gripDevice.set(.6);
        } 
        if(buttons.getRawButtonPressed(12)){
            activated = false;
        }
        if(!activated){
            gripDevice.set(0);
        }
    }
    

}

