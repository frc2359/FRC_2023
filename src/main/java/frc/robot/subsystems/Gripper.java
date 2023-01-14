package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.RobotMap.*;

public class Gripper {
    private CANSparkMax extMotor0 = new CANSparkMax(GRIPPER_0, MotorType.kBrushless);
    private CANSparkMax extMotor1 = new CANSparkMax(GRIPPER_1, MotorType.kBrushless);
    private CANSparkMax extMotors[] = new CANSparkMax[]{extMotor0, extMotor1};
    PIDController pid = new PIDController(0.00000000001, 0.00000000001, 0.00000000001);

    

}

