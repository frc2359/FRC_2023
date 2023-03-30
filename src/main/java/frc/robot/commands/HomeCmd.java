package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lifter;
import frc.robot.IO;
import static frc.robot.RobotMap.*;

public class HomeCmd extends CommandBase{
    
    private Lifter lift;
    private Gripper grip;
    private Extender extend;
    private int liftDist;
    private int liftDeadband;
    private int extenderDist;
    private int extenderDeadband;
    private int foo;


    public void home(){
        if(IO.isLeftAxisPressed()){
            extend.zeroPosition();
            lift.zeroPosition();
        }
    }

}
