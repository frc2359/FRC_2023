package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;

import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.ExtenderConstants.*;

public class Extender {
    private static TalonSRX extendMot;
    // private int state = 0;
    private int stateExtender = -1;
    private double spdExtender = 0;
    private double targetDistanceInches = 0;
    private double userControl = 0;
    private double distance = 0;
    private boolean autoCompleted = false;

    public void init() {
        extendMot = new TalonSRX(EXTEND_MOT_ID);
        extendMot.configFactoryDefault();
        // extendMot.configForwardLimitSwitchSource(null, null);
        extendMot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);
        extendMot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        extendMot.setInverted(false);
        extendMot.setSelectedSensorPosition(0);
    }

    /**Set Extender State */
    public void setStateExtender(int st) {
        this.stateExtender = st;
    }

    public boolean extendToDistance(double dist) {
        this.distance = dist;
        this.stateExtender = CASE_EXTEND_TO_DIST;
        // extendEm();
        return autoCompleted;
    }

    public double getDistanceInches() {
        return setGlobalDistanceInches(extendMot.getSelectedSensorPosition() / 195.44);
    }

    private double setGlobalDistanceInches(double dist) {
        currentlyExtended = dist;
        return dist;
    }

    public double convertToRawDistance(double dist) {
        return dist * 195.44;
    }

    /* TEMP */
    public void setExtUnknown() {
        stateExtender = STATE_EXT_UNKNOWN;
    }

    public void zeroPosition() {
        stateExtender = STATE_EXT_UNKNOWN;
        runExtender();
    }

    public double getPosition() {
        return extendMot.getSelectedSensorPosition();
    }

    public void showData() {
        SmartDashboard.putBoolean("Extender Back", isHome());
        SmartDashboard.putBoolean("Extender Fwd", extendMot.getSensorCollection().isRevLimitSwitchClosed());
        SmartDashboard.putBoolean("Extender Rev", extendMot.getSensorCollection().isFwdLimitSwitchClosed());
    }

    /**Case system that directly controls movement */
    public void runExtender() {
        spdExtender = 0;
        userControl = IO.getLiftControlRightX();

        if(isHome()) {  setEncoderHome();  }

        SmartDashboard.putNumber("Extender Encoder", extendMot.getSelectedSensorPosition());
        SmartDashboard.putNumber("Extender Inches", getDistanceInches());
        SmartDashboard.putNumber("Extender Case", stateExtender);
        SmartDashboard.putBoolean("Extender Back", isHome());
        SmartDashboard.putBoolean("Extender Fwd", extendMot.getSensorCollection().isRevLimitSwitchClosed());
        SmartDashboard.putBoolean("Extender Rev", extendMot.getSensorCollection().isFwdLimitSwitchClosed());
        SmartDashboard.putNumber("UserCtrl",userControl);
        


        if (Math.abs(userControl) < 0.1) { // deadband
            userControl = 0.0; 
        }    

        
        switch(stateExtender) {
            case STATE_EXT_UNKNOWN:
                spdExtender = -0.7;      // retract until limit switch is reached
                if(isHome()) {
                    spdExtender = 0;
                    userControl = 0;
                    setEncoderHome();
                    stateExtender = STATE_EXT_STOP;
                }
                break; 

            case STATE_EXT_STOP:
                spdExtender = 0;
                if (userControl > 0) {
                    stateExtender = STATE_EXT_EXTEND;
                } else if (userControl < 0) {
                    stateExtender = STATE_EXT_RETRACT;
                } else { 
                    stateExtender = STATE_EXT_STOP;
                }
                break;
            
            case STATE_EXT_EXTEND:
                if (getDistanceInches() >= EXTENDER_MAX_DISTANCE) {
                    spdExtender = 0;
                    stateExtender = STATE_EXT_STOP;
                    break;
                } else if (getDistanceInches() > EXTENDER_MAX_DISTANCE - EXTENDER_SLOW_DISTANCE) {
                    spdExtender = EXTENDER_SLOW_SPEED * userControl;
                } else {
                    spdExtender = EXTENDER_FAST_SPEED * userControl;
                }
                if (userControl > 0) {
                    stateExtender = STATE_EXT_EXTEND;
                    break;
                } else if (userControl < 0) {
                    stateExtender = STATE_EXT_RETRACT;
                } else { stateExtender = STATE_EXT_STOP;}
                break;
            
            case STATE_EXT_RETRACT:
                SmartDashboard.putNumber("Extender Case", stateExtender);

                if (isHome()) {
                    spdExtender = 0;
                    stateExtender = STATE_EXT_STOP;
                } else if (getDistanceInches() < EXTENDER_SLOW_DISTANCE) {
                    spdExtender = EXTENDER_SLOW_SPEED * userControl;
                } else {
                    spdExtender = EXTENDER_FAST_SPEED * userControl;
                }
                if (userControl > 0) {
                    stateExtender = STATE_EXT_EXTEND;
                } else if (userControl < 0) {
                    stateExtender = STATE_EXT_RETRACT;
                } else { stateExtender = STATE_EXT_STOP;}
                break;
            
            case STATE_EXT_MOVE_TO_POS:
                double pos = getDistanceInches();
                if (Math.abs(pos - targetDistanceInches) < 0.05 || userControl != 0.0) {
                    spdExtender = 0.0;
                    stateExtender = STATE_EXT_STOP;
                    break;
                }
                if (Math.abs(pos - targetDistanceInches) > EXTENDER_SLOW_DISTANCE) {
                    spdExtender = EXTENDER_FAST_SPEED;
                } else {
                    spdExtender = EXTENDER_SLOW_SPEED;
                }
                if (targetDistanceInches < pos) {
                    spdExtender *= -1.0;
                }
                break;
            /*
            case CASE_EXTEND_TO_DIST:
                if(extendMot.getSelectedSensorPosition() >= distance) {
                    extendMot.set(ControlMode.PercentOutput, 0);
                    autoCompleted = true;
                } else {
                    extendMot.set(ControlMode.PercentOutput, .3);
                }
                break;
            */
        }

        SmartDashboard.putNumber("spdExt",spdExtender);

        extendMot.set(ControlMode.PercentOutput, spdExtender);
    }

    /** Move Extender to Pos */
    public void setToDistance(double dist) {
        if (dist < 0.0) { dist = 0;}
        if (dist > EXTENDER_MAX_DISTANCE) { dist = EXTENDER_MAX_DISTANCE;}
        targetDistanceInches = dist;
        if (stateExtender != STATE_EXT_UNKNOWN) { stateExtender = STATE_EXT_MOVE_TO_POS; }
    }

    /** Checks if home limit switch enabled */
    public boolean isHome() {
        if(extendMot.getSensorCollection().isRevLimitSwitchClosed() || extendMot.getSensorCollection().isFwdLimitSwitchClosed()) {
            extendMot.setSelectedSensorPosition(0);
            
            return true;
        } else {
            return false;
        }
    }
    

    /* reset encoder */
    public void setEncoderHome() {
        extendMot.setSelectedSensorPosition(0);
    }
}
