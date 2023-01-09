 //package
package frc.robot.subsystems;

//subsystem interface
import edu.wpi.first.wpilibj2.command.Subsystem;

//other imports

public class Drive implements Subsystem {

    //wheel objects
    Wheel a;
    Wheel b;
    Wheel c;
    Wheel d;

    //nested wheel class (for abstraction)
    public class Wheel {
        
        //constructor
        public Wheel() {

            //interface to wheel

        }

        //get speed

        //set speed

        //get angle

        //set angle

    }

}