package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoPathCmd;
import frc.robot.commands.LifterCommands;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lifter;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private int countLoop;

    private RobotContainer m_robotContainer;
    private Lifter lift;
    private Gripper gripper;
    private Extender extender;

    private LifterCommands lifterCommands = new LifterCommands();
    private AutoPathCmd apc = new AutoPathCmd();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    int autoMode = 1;

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        gripper = new Gripper();
        lift = new Lifter();
        extender = new Extender();
        lift.init();
        gripper.init();
        extender.init();
        SmartDashboard.putNumber("autoMode", autoMode);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods. This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        
        // SmartDashboard.getNumber("autoMode", autoMode);
        if( SmartDashboard.getNumber("autoMode", autoMode) == 1){
            m_autonomousCommand = lifterCommands.runLiftExtend(lift, extender, 2, 2).andThen(m_robotContainer.runPath("New Path", 8, 7));
        } else if (SmartDashboard.getNumber("autoMode", autoMode) == 2) {
            HashMap<String, Command> events = new HashMap<>();
            events.put("putDownCone", lifterCommands.print("we good?"));
            events.put("balance", lifterCommands.print("no but like actually?"));
            m_autonomousCommand = m_robotContainer.runPathWithEvents("New Event Path", 3, 2, events);
        }
       

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

           
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
        lift.init();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        countLoop = 0;

        extender.setExtUnknown();
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods. This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        lift.manualRun();
        // gripper.manualRun();
        extender.runExtender();
        if(IO.getButton(12)) {
            extender.setToDistance(10);
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}