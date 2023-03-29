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

public class LiftThrowCmd extends CommandBase {
    private Lifter lift;
    private Gripper grip;
    private Extender extend;
    private int liftDist;
    private int liftDeadband;
    private int extenderDist;
    private int extenderDeadband;
    private int foo;

    public LiftThrowCmd(Lifter lift, Gripper grip, Extender extend, int liftDist, int liftDeadband) {
        this.lift = lift;
        this.grip = grip;
        this.extend = extend;
        this.liftDist = liftDist;
        this.liftDeadband = liftDeadband;
    }

    @Override
    public void execute() {
        int count = 0;
        boolean contnue = false;

        switch(foo) {
            case 0:
                lift.setState(LifterConstants.STATE_LIFT_UNKOWN);
                lift.run();
                if(lift.isHomed()){
                    foo++;
                }
                break;

            case 1:
                contnue = lift.autoRun(liftDist, liftDeadband); // 41
                if (contnue) {
                    foo++;
                    contnue = false;
                }
                break;
            
                case 2:
                while (count < 10) {
                    grip.setState(ClawConstants.CASE_EXPEL_CUBE_HIGH).run();
                    count++;
                }
                if (count >= 10) {
                    grip.setState(ClawConstants.CASE_STOP).run();
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        lift.zeroPosition();
    }

    @Override
    public boolean isFinished() {
        if (lift.isHomed() && foo == 2) {
            return false;
        } else {
            return true;
        }
    }

}
