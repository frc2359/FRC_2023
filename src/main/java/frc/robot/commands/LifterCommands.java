package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lifter;
import static frc.robot.RobotMap.*;

public class LifterCommands  extends CommandBase {
    public Command print(String printStr) {
        return new InstantCommand(() -> {
            System.out.println(printStr);
            SmartDashboard.putBoolean(printStr, true);
        });
    }

    public Command liftThrow(Lifter lift, Gripper grip, int liftDist, int deadband) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                int num = 0;
                int count = 0;
                boolean contnue = false;

                switch(num) {
                    case 0:
                        SmartDashboard.putNumber("kjahflkja", num);
                        contnue = lift.autoRun(liftDist, deadband);
                        if (contnue) {
                            num = 1;
                            contnue = false;
                        }
                        break;
                    case 1:
                        SmartDashboard.putNumber("kjahflkja", num);
                        while (count < 10) {
                            grip.setState(ClawConstants.CASE_EXPEL_CUBE_HIGH).run();
                            count++;
                        }
                        if (count >= 10) {
                            grip.setState(ClawConstants.CASE_STOP).run();
                        }
                        break;
                }
            })
        );
    }

    public boolean execute() {

    }

    /**Extend the extender, then run the lifter to the desired locations
     * @param lift is the lifter object being targeted
     * @param extender is the extender object being targeted
     * @param extenderDist is the desired distance to extend
     * @param liftDistance is the desired rotation of the lifter
     */
    public Command runLiftExtend(Lifter lift, Extender extender, int extenderDist, int liftDistance) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                System.out.println("we good?");
                int num = 0;
                boolean contnue = false;
                switch(num) {
                    case 0:
                        contnue = extender.extendToDistance(extenderDist);
                        if(contnue) {
                            contnue = false;
                            num++;
                        }
                        break;
                    case 1:
                        contnue = lift.autoRun(liftDistance, 5);
                        break;
                        

                }
            })
        );
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
    }
}
