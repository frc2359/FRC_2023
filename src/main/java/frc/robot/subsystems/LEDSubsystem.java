package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.subsystems.Gripper;


import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.LEDConstants.*;

import javax.lang.model.util.ElementScanner14;


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

    private static Gripper myGripper;
    private static Lifter myLifter;
    private static Extender myExtender;

    private static int pieceType = LEDConstants.PIECE_TYPE_UNKNOWN;

     private void waitMSecs(long delay) {
        startTime = System.currentTimeMillis();
        elapsedTime = 0;
        while (elapsedTime < delay) {
            elapsedTime = System.currentTimeMillis() - startTime;
        }
    }

    public void init(Gripper grp, Lifter lft, Extender ext) {
        myGripper = grp;
        myLifter = lft;
        myExtender = ext;
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
        //leds.setData(ledBuffer);
    }

    public void initLEDs() {
        /*
        setColor(1, 61, 0, 0, 0);
        for (int i = 1; i<=61 ; i++) {
            setColor(1, i, 255,255,255);
            waitMSecs(10);
            //setColor(0, i, i, 0,0,0);
        }
        */
        setState(STATE_LEDS_INIT);
        runLEDs();
    }

    public void setState (int st) {
        stateLEDs = st;
    }

    public  void setPiece (int pT) {
        pieceType = pT;
        setState(STATE_LEDS_PIECE);
    }


    public void setPair(int iP, int cR, int cG, int cB) {
        setColor(iP, iP, cR, cG, cB);
        setColor(62-iP, 62-iP, cR, cG, cB);
        //ledBuffer.setRGB(iP-1, cR, cG, cB);
        //ledBuffer.setRGB(61-iP, cR, cG, cB);
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
                stateLEDs = STATE_LEDS_OFF;
                break;
            case STATE_LEDS_STATUS:

                for (int i=0; i<61; i++) {
                    ledBuffer.setRGB(i, 0, 0, 0);
                }
                double batVal = IO.getBattVoltage();
                setPair(5, 0, 0, 0);
                setPair(6, 0, 0, 0);
                
                if (batVal > 12.5) {
                    setPair(5, 0, 255, 0);
                    setPair(6, 0, 0, 255);
                }
                if (batVal >= 12.0 && batVal <12.5 ) {
                    setPair(5, 0, 255, 0);
                    setPair(6, 0, 255, 0);
                }
                if (batVal >= 11.5 && batVal <12.0 ) {
                    setPair(5, 0, 255, 0);
                    setPair(6, 255, 255, 0);
                }
                if (batVal >= 11.0 && batVal <11.5 ) {
                    setPair(5, 255, 255, 0);
                    setPair(6, 255, 255, 0);
                }
                if (batVal <11.0 ) {
                    setPair(5, 255, 0, 0);
                    setPair(6, 255, 0, 0);
                }

                // Status Info

                if (DriverStation.isEnabled()) {
                    setPair(3, 0,255, 0);
                    if (DriverStation.isAutonomous()) {
                        setPair(4, 255, 255, 0);
                    }
                    if (DriverStation.isTeleop()) {
                        setPair(4, 0, 255, 0);
                    }
                    if (DriverStation.isTest()) {
                        setPair(4, 255, 255, 255);
                    }
                    setPair(5, 0,255, 0); 
                }
                if (DriverStation.isDisabled()) {
                    setPair(7, 128,128, 0);
                    setPair(8, 128,128, 0);
                    setPair(9, 128,128, 0); 
                }
                if (DriverStation.isEStopped()) {
                    setPair(7, 255,0, 0);
                    setPair(8, 255,0, 0);
                    setPair(9, 255,0, 0); 
                }

                // Alliance Info
                int stn = DriverStation.getLocation();
                for (int i = 0; i<stn; i++) {
                    if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                        setPair(10+i,255,0,0);
                    }
                    else {
                        setPair(10+i,0,0,255);
                    }
                }
                for (int i=stn; i<3; i++) {
                    setPair(11+i, 255, 255, 255);
                }
                
                if (DriverStation.isFMSAttached()) {
                    setPair(15, 0, 255, 0);
                } else {
                    setPair(15, 255, 0, 0);
                }

                if (DriverStation.isDSAttached()) {
                    setPair(17, 0, 255, 0);
                } else {
                    setPair(17, 255, 0, 0);
                }

                if (IO.isNavXAvail()) {
                    setPair(19, 0, 255, 0);
                } else {
                    setPair(19, 255, 0, 0);
                }

                if (myLifter.currentlyHomed()) {
                    setPair(21, 0, 255, 0);
                } else {
                    setPair(21, 255, 0, 0);
                }

                if (myExtender.isHome()) {
                    setPair(23, 0, 255, 0);
                } else {
                    setPair(23, 255, 0, 0);
                }

                if (myGripper.hasCube()) {
                    setPair(25, 0, 255, 0);
                    setPair(26, 255, 0, 255);
                    setPair(27, 0, 255, 0);
                } else if (myGripper.hasCone()) {
                    setPair(25, 0, 255, 0);
                    setPair(26, 255, 128, 0);
                    setPair(27, 0, 255, 0);
                } else {
                    setPair(25, 255, 0, 0);
                    setPair(26, 255, 0, 0);
                    setPair(27, 255, 0, 0);
                }

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
                if (pieceType == LEDConstants.PIECE_TYPE_CUBE) {
                    setColor(1, 61, 255, 0, 255);
                    if (myGripper.hasCube()) {
                        setColor(21, 41, 0, 255, 0);
                    }
                    if (myGripper.hasCone()) {
                        setColor(21, 41, 255, 0, 0);
                    }
                } else if (pieceType == LEDConstants.PIECE_TYPE_CONE) {
                    setColor(1, 61, 255, 128, 0);
                    if (myGripper.hasCone()) {
                        setColor(21, 41, 0, 255, 0);
                    }
                    if (myGripper.hasCube()) {
                        setColor(21, 41, 255, 0, 0);
                    }
                } else {setColor(1, 61, 0, 0, 0);}
                break;

            case STATE_LEDS_AUTO:
                break;
                
            case STATE_LEDS_COUNTDOWN:
                break;        
        }
       leds.setData(ledBuffer);
    }
}