package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.RobotMap.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Arduino {
    
    //private DigitalInput sensorBallRed = new DigitalInput(3);
    //private DigitalInput sensorBallBlue = new DigitalInput(4);

    private SerialPort arduinoUSB;
    private Timer timer; 

    private String cmd;
    private boolean arduinoFound;

    //private final I2C.Port i2cPort = I2C.Port.kOnboard;
    //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    //private final I2C arduinoI2C = new I2C(Port.kMXP,4);

    // private final ColorMatch m_colorMatcher = new ColorMatch();
    // private final Color kBlueTarget = new Color(0.19, 0.427, 0.38);
    // private final Color kRedTarget = new Color(0.456, 0.378, 0.166);
    // private final Color kBlankTarget = new Color(0.273, 0.479, 0.240);
    
    // private int ballColor;


    /** Arduino Init */
    public void init() {

        // m_colorMatcher.addColorMatch(kBlueTarget);
        // m_colorMatcher.addColorMatch(kRedTarget);

        
        // Establish Connection to Arduino
        try {
            arduinoUSB = new SerialPort(9600, SerialPort.Port.kUSB);
            System.out.println("Connected on kUSB!");
            arduinoFound = true;
        } catch (Exception e) {
            System.out.println("Failed to connect on kUSB, trying kUSB1");
            try {
                arduinoUSB = new SerialPort(9600, SerialPort.Port.kUSB1);
                System.out.println("Connected on kUSB1!");
                arduinoFound = true;
            } catch (Exception e1) {
                System.out.println("Failed to connect on kUSB1, trying kUSB2");
                try {
                    arduinoUSB = new SerialPort(9600, SerialPort.Port.kUSB2);
                    System.out.println("Connected on kUSB2!");
                    arduinoFound = true;
                } catch (Exception e2) {
                    System.out.println("Failed to connect on kUSB2, all attempts failed.");
                    arduinoFound = false;
                }
            }
         }
        
        SmartDashboard.putBoolean("Arduino", arduinoFound);
        String testString = "Init";
        System.out.print("Init nessage - ");
        // System.out.println(writeArduino(testString));
    }  
    
    public boolean writeArduino (String toSend) {
        //boolean success = arduinoI2C.transaction(toSend.getBytes(), toSend.getBytes().length, new byte[0], 0);
        return true; //success;
    }
     
    public int getBallColor() {     
            
        //Color detectedColor = m_colorSensor.getColor();

        /**
         * Run the color match algorithm on our detected color
         */
        String colorString = "Unknown";
        /*
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
        if (match.color == kBlueTarget && detectedColor.blue > .37) {
          colorString = "Blue";
          ballColor = COLOR_BLUE;
        } else if (match.color == kRedTarget && detectedColor.red > .44) {
          colorString = "Red";
          ballColor = COLOR_RED;
        } else if (match.color == kBlankTarget || detectedColor.green > .47) {
            colorString = "Empty";
            ballColor = COLOR_UNKNOWN;
        } else {
          colorString = "Unknown";
          ballColor = COLOR_UNKNOWN;
        }
        /*
    
        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        /*
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Conf.", match.confidence);
        SmartDashboard.putString("Detected", colorString);
        */

        int COLOR_UNKNOWN = 0; //placeholder to avoid errors from copied code.
        
        return COLOR_UNKNOWN; //ballColor;
        

    }


    public boolean isArduinoFound() {
        return arduinoFound;
    }

    /**  LED CONTROL */
    public void defineLEDString (int port, int numLEDs) {
       // send to Arduino the port and numLEDs
        cmd = "D"+Integer.toString(port)+","+Integer.toString(numLEDs);
        //System.out.println(cmd);
        if (arduinoFound) {
            arduinoUSB.writeString(cmd);
            arduinoUSB.write(new byte[]{0x0A}, 1);
        }
     }
    
    public void setLEDColor (int port, int colorCode) {
        // send port and colorCode to Arduino
        cmd = "S"+Integer.toString(port)+","+Integer.toString(colorCode);
        //System.out.println(cmd);
        if (arduinoFound) {
            arduinoUSB.writeString(cmd);
            arduinoUSB.write(new byte[]{0x0A}, 1);
        }
    }
}
