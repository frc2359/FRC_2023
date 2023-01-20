package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.rev.NeoDriveControllerFactoryBuilder;

import edu.wpi.first.math.controller.PIDController;

public class WheelDrive {
    
    private Jaguar angleMotor;
    private Jaguar speedMotor;
    private PIDController pidController;

    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new Jaguar (angleMotor);
        this.speedMotor = new Jaguar (speedMotor);
        pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), this.angleMotor);
    
        pidController.setOutputRange (-1, 1);
        pidController.setContinuous ();
        pidController.enable ();
    }

}
