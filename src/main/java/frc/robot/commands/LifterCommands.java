package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Lifter;

public class LifterCommands  extends SequentialCommandGroup {
    public Command runLiftExtend(Lifter lift, Extender extender) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                int num = 0;
                boolean contnue = false;
                switch(num) {
                    case 0:
                        contnue = extender.extendToDistance(5);
                        if(contnue) {
                            contnue = false;
                            num++;
                        }
                        break;
                    case 1:
                        contnue = lift.autoRun(5);
                        break;
                        

                }
                extender.extendToDistance(5);
                lift.autoRun(5);
            })
        );
    }
}
