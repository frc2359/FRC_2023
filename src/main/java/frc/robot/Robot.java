package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
            m_autonomousCommand = m_robotContainer.runPath();
        }
       

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
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
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
        lift.init();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        countLoop = 0;
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods. This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        lift.manualRun();
        gripper.manualRun();
        extender.manualRun();
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