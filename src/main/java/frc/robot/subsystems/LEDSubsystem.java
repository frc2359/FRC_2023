package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.LEDConstants.*;


public class LEDSubsystem extends SubsystemBase {
    
    private AddressableLED ledLeft;
    private AddressableLEDBuffer ledLeftBuffer;
   // private AddressableLED ledRight;
   // private AddressableLEDBuffer ledRightBuffer;

    private long startTime = 0;
    private long elapsedTime = 0;

    private void waitMSecs(long delay) {
        startTime = System.currentTimeMillis();
        elapsedTime = 0;
        while (elapsedTime < delay) {
            elapsedTime = System.currentTimeMillis() - startTime;
        }
    }

    public void init() {
        ledLeft = new AddressableLED(PWM_LED_LEFT);
        ledLeftBuffer = new AddressableLEDBuffer(61);
        ledLeft.setLength(ledLeftBuffer.getLength());
        ledLeft.setData(ledLeftBuffer);
        ledLeft.start();
        
        // ledRight = new AddressableLED(PWM_LED_RIGHT);
        // ledRightBuffer = new AddressableLEDBuffer(61);
        // ledRight.setLength(ledRightBuffer.getLength());
        // ledRight.setData(ledRightBuffer);
        // ledRight.start();
    }

    /** */
    public void setColor(int arg1, int iStart, int iEnd, int rCol, int gCol, int bCol) {
        for (int i = iStart -1; i < iEnd; i++) {
            if(arg1 == 1 || arg1 == 0) {
                ledLeftBuffer.setRGB(i, rCol, gCol, bCol);
            }
            if (arg1 == 2 || arg1 == 0) {
                // ledRightBuffer.setRGB(i, rCol, gCol, bCol);
            }
        }
        ledLeft.setData(ledLeftBuffer);
        // ledRight.setData(ledRightBuffer);
    }

    public void initLEDs() {
        setColor(0, 1, 61, 0, 0, 0);
        for (int i = 1; i<=61 ; i++) {
            setColor(0, 1, i, 255,255,255);
            waitMSecs(10);
            //setColor(0, i, i, 0,0,0);
        }
    }
}