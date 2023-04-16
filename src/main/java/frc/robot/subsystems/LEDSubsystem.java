package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.LEDConstants.*;


public class LEDSubsystem extends SubsystemBase {

    private int stateLEDs = STATE_LEDS_OFF;

    private int colR = 0;
    private int colG = 0;
    private int colB = 0;
    private boolean colBlink = false;
    private int colDelay = 500;
    
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
   
    private long startTime = 0;
    private long elapsedTime = 0;

    private DriverStation dS;

    private void waitMSecs(long delay) {
        startTime = System.currentTimeMillis();
        elapsedTime = 0;
        while (elapsedTime < delay) {
            elapsedTime = System.currentTimeMillis() - startTime;
        }
    }

    public void init() {
        leds = new AddressableLED(PWM_LEDS);
        ledBuffer = new AddressableLEDBuffer(61);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    public void setCol(int cR, int cG, int cB, boolean blink) {
        colR = cR;
        colG = cG;
        colB = cB;
        colBlink = blink;
        stateLEDs = STATE_LEDS_COLOR;
    }

    /** */
    private void setColor(int iStart, int iEnd, int rCol, int gCol, int bCol) {
        for (int i = iStart -1; i < iEnd; i++) {
            ledBuffer.setRGB(i, rCol, gCol, bCol);
        }
        leds.setData(ledBuffer);
    }

    public void initLEDs() {
        setColor(1, 61, 0, 0, 0);
        for (int i = 1; i<=61 ; i++) {
            setColor(1, i, 255,255,255);
            waitMSecs(10);
            //setColor(0, i, i, 0,0,0);
        }
    }

    public void setState (int st) {
        stateLEDs = st;
    }

    public void setPair(int iP, int cR, int cG, int cB) {
        //setColor(iP, iP, cR, cG, cB);
        //setColor(62-iP, 62-iP, cR, cG, cB);
        ledBuffer.setRGB(iP, cR, cG, cB);
        ledBuffer.setRGB(62-iP, cR, cG, cB);
    }

    public void runLEDs() {
        switch (stateLEDs) {
            case STATE_LEDS_OFF:
                setColor(1, 61, 0, 0, 0);
                break;
            case STATE_LEDS_INIT:
                setColor(1, 61, 0, 0, 0);
                for (int i = 1; i<=61 ; i++) {
                    setColor(1, i, 255,255,255);
                    waitMSecs(10);
                    //setColor(0, i, i, 0,0,0);
                }
                break;
            case STATE_LEDS_STATUS:

                //setColor(1, 61, 0, 0, 0);
                double batVal = 12.1;
                setPair(1, 0, 0, 0);
                setPair(2, 0, 0, 0);
                
                if (batVal > 12.5) {
                    setPair(1, 0, 255, 0);
                    setPair(2, 0, 0, 255);
                }
                if (batVal >= 12.0 && batVal <12.5 ) {
                    setPair(1, 0, 255, 0);
                    setPair(2, 0, 255, 0);
                }
                if (batVal >= 11.5 && batVal <12.0 ) {
                    setPair(1, 0, 255, 0);
                    setPair(2, 255, 255, 0);
                }
                if (batVal >= 11.0 && batVal <11.5 ) {
                    setPair(1, 255, 255, 0);
                    setPair(2, 255, 255, 0);
                }
                if (batVal <11.0 ) {
                    setPair(1, 255, 0, 0);
                    setPair(2, 255, 0, 0);
                }
/*
                // Status Info

                if (DriverStation.isEnabled()) {
                    setPair(3, 0,255, 0);
                    setPair(5, 0,255, 0); 
                    if (DriverStation.isAutonomous()) {
                        setPair(4, 255, 255, 0);
                    }
                    if (DriverStation.isTeleop()) {
                        setPair(4, 0, 255, 0);
                    }
                    if (DriverStation.isTest()) {
                        setPair(4, 255, 255, 255);
                    }
                }
                if (DriverStation.isDisabled()) {
                    setPair(3, 255,255, 0);
                    setPair(4, 255,255, 0);
                    setPair(5, 255,255, 0); 
                }
                if (DriverStation.isEStopped()) {
                    setPair(3, 255,0, 0);
                    setPair(4, 255,0, 0);
                    setPair(5, 255,0, 0); 
                }

                // Alliance Info
                setPair(6, 255,255,255);
                setPair(7, 255,255,255);
                setPair(8, 255,255,255);
                int stn = DriverStation.getLocation();
                for (int i = 0; i<stn; i++) {
                    if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                        setPair(6+i,255,0,0);
                    }
                    else {
                        setPair(6+1,0,0,255);
                    }
                    //setPair(3+i,(IO.isTeamRed() ? 255 : 0), 0, (IO.isTeamRed() ? 0 : 255));
                }
                */
                break;
            case STATE_LEDS_COLOR:
                    setColor(1, 61, colR, colG, colB);
                    if (colBlink) {
                        waitMSecs(colDelay);
                        setColor(1, 61, 0, 0, 0);
                        waitMSecs(colDelay);
                    }
                break;
            case STATE_LEDS_PIECE:
                break;
            case STATE_LEDS_AUTO:
                break;
            case STATE_LEDS_COUNTDOWN:
                break;        
        }
       // leds.setData(ledBuffer);
    }
}