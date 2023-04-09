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
import static frc.robot.RobotMap.AutoConstants.*;

public class LiftThrowCmd {
    private Lifter lift;
    private Gripper grip;
    private Extender extend;
    private int liftDist;
    private int liftDeadband;
    private int extenderDist;
    private int extenderDeadband;
    private int liftCmdState;
    private int liftThrowCase = 0;

    public LiftThrowCmd(Lifter lift, Gripper grip, Extender extend, int liftDist, int liftDeadband) {
        this.lift = lift;
        this.grip = grip;
        this.extend = extend;
        this.liftDist = liftDist;
        this.liftDeadband = liftDeadband;
        // lift.setState(LifterConstants.STATE_LIFT_UNKOWN);
    }

    public LiftThrowCmd(Lifter lift, Gripper grip, Extender extend) {
        this.lift = lift;
        this.grip = grip;
        this.extend = extend;
    }

    public void run(int liftThrowCase) {
        switch(liftThrowCase) {
            case kZero:
                extend.setToDistance(0, 0.2);
                lift.autoRun(0, 3);
                break;
            case kDown:
                extend.setToDistance(0, 0.2);
                lift.autoRun(LifterConstants.LIFTER_MAX_ROTATION - 5, 5);
                break;
            case kMidCube:
                extend.setToDistance(0, 0.2);
                lift.autoRun(68, 5);
                break;
            case kHighCube:
                extend.setToDistance(0, 0.2);
                lift.autoRun(41, 5);
                break;
            case kCollectLow:
                extend.setToDistance(0, 0.2);
                lift.autoRun(LifterConstants.LIFTER_MAX_ROTATION - 10, 5);
                grip.setState(ClawConstants.CASE_INTAKE);
                break;
            case kChute:
                extend.setToDistance(0, 0.2);
                lift.autoRun(60, 5);
                break;
            case kSubstation:
                extend.setToDistance(10, 0.5);
                lift.autoRun(70, 0.2);
                break;
            case kLow:
                extend.setToDistance(0, 0.2);
                lift.autoRun(LifterConstants.LIFTER_MAX_ROTATION - 10, 3);
                break;
            
        }
    }

}
