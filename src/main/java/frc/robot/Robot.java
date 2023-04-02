package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO.GyroType;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.ClawConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ExtenderConstants;
import frc.robot.RobotMap.LifterConstants;
import frc.robot.RobotMap.LimelightConsants;
import frc.robot.commands.AutoPathCmd;
import frc.robot.commands.LiftThrowCmd;
// import frc.robot.subsystems.Arduino;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private int countLoop;

    private RobotContainer m_robotContainer;
    private Lifter lift;
    private Gripper gripper;
    private Extender extender;
    // private Arduino arduino;

    private LiftThrowCmd lifterCommands;

    private PowerDistribution pdh = new PowerDistribution();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    int autoMode = 2;



    int num = 0;
    int count = 0;

    int countPressTime = 0;
    int countGripTime = 0;

    int liftCmdState = 0;

    boolean contnue = false;

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        gripper = new Gripper();
        lift = new Lifter();
        extender = new Extender();
        // arduino = new Arduino();

        lift.init();
        gripper.init();
        // arduino.init();
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

        SmartDashboard.putNumber("Battery Voltage", (pdh.getVoltage()));
        IO.getAprilTagValues();
        SmartDashboard.putBoolean("Battery Good (> 11.5)", (pdh.getVoltage() > 11.5));
        m_robotContainer.getSwerveSubsystem().showData();
    }

    @Override
    public void disabledPeriodic() {
        IO.setLed(LimelightConsants.kLedOff);
        m_robotContainer.getSwerveSubsystem().setDriveMode(DriveConstants.kCoastMode);
        extender.showData();
        lift.setDriveMode(false);
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        IO.setLed(LimelightConsants.kPipelineLedSettings);
        lift.init();
        extender.init();

        lift.setDriveMode(true);

        countGripTime = 0;
        liftCmdState = 0;

        lift.setStateLifter(LifterConstants.STATE_LIFT_UNKOWN);

        m_robotContainer.getSwerveSubsystem().setDriveMode(DriveConstants.kBrakeMode);


        // lifterCommands = new LiftThrowCmd(lift, gripper, extender, 42, 5);

        SmartDashboard.getNumber("autoMode", autoMode);

        if(SmartDashboard.getNumber("autoMode", autoMode) == AutoConstants.kOut) {
            m_autonomousCommand = m_robotContainer.runPath("Out", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
        } else if (SmartDashboard.getNumber("autoMode", autoMode) == AutoConstants.kShootAndFarOutSpin) {
            m_autonomousCommand = m_robotContainer.runPath("Out Far Spin", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
        } else if (SmartDashboard.getNumber("autoMode", autoMode) == AutoConstants.kShootAndOut || SmartDashboard.getNumber("autoMode", autoMode) == AutoConstants.kShootOutBalance) {
            m_autonomousCommand = m_robotContainer.runPath("Out", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
        } else if (SmartDashboard.getNumber("autoMode", autoMode) == 5) {
            liftCmdState = 6;
        }
       

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null && autoMode == 1) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("Auto State", liftCmdState);
        if (autoMode >= AutoConstants.kShootAndFarOutSpin || autoMode <= AutoConstants.kShootOutBalance) {
            switch(liftCmdState) {
                case 0:
                    if(lift.currentlyHomed()){
                        liftCmdState++;
                    }
                    lift.run();
                    break;
                case 1:
                    // contnue = lift.autoRun(68, 5); // 41
                    contnue = lift.autoRun(41, 5); // 41

                    if (contnue) {
                        liftCmdState++;
                        contnue = false;
                    }
                    break;
                case 2:
                    if (countGripTime < 50) {
                        gripper.setState(ClawConstants.CASE_EXPEL_CUBE_HIGH).run();
                        countGripTime++;
                    }
                    if (countGripTime >= 50) {
                        gripper.setState(ClawConstants.CASE_STOP).run();
                        liftCmdState++;
                        // countGripTime = 0;
                    }
                    break;
                case 3:
                    // lift.zeroPosition();
                    liftCmdState++;
                    break;
                case 4:
                    m_autonomousCommand.schedule();
                    liftCmdState++;
                case 5:
                    if (m_autonomousCommand.isFinished() && autoMode == AutoConstants.kShootOutBalance) {
                        m_autonomousCommand = m_robotContainer.runPath("Out", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
                        m_autonomousCommand.schedule();
                        liftCmdState++;
                    } else {
                        System.out.println("done");
                    }
                    break;
                case 6:
                    if(Math.abs(IO.getPitch()) > 0.5) {
                        m_autonomousCommand.cancel();
                        m_robotContainer.balance();
                    }
                    break;
            }
        }

        
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
        lift.init();
        
        IO.setLed(LimelightConsants.kPipelineLedSettings);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.getSwerveSubsystem().setDriveMode(true);
        lift.setDriveMode(true);
        lift.setStateLifter(LifterConstants.STATE_LIFT_STOP);
        extender.setStateExtender(ExtenderConstants.STATE_EXT_STOP);

        
        countLoop = 0;
        countPressTime = 0;


        extender.setExtUnknown();

        
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods. This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        lift.manualRun();
        gripper.manualControl();
        extender.runExtender();
        if(IO.getDriverButton(12)) {
            extender.setToDistance(10);
        }

        if (IO.getDriverButton(3)) { //68
            m_robotContainer.getSwerveSubsystem().setDriveMode(true);
        } else if (IO.getDriverButton(4)) {
            m_robotContainer.getSwerveSubsystem().setDriveMode(false);
        }

        if (IO.isLeftAxisPressed()){
            extender.setToDistance(0);
            lift.setToDistance(5, 5);
        }

        if(IO.isRightAxisPressed()) {
            extender.setToDistance(0);
            lift.setToDistance(LifterConstants.LIFTER_MAX_ROTATION - 5, 5);
        }

        if (IO.isLeftBumpPressed()) {
            extender.setToDistance(0);
            lift.setToDistance(68, 5);
        }

        if(IO.getDriverButton(10)) {
            IO.zeroHeading();
        }

        // if (countPressTime == 50 && )
        

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