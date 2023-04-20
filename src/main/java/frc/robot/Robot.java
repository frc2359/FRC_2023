package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO.GyroType;
import frc.robot.RobotMap.AutoConstants;
import static frc.robot.RobotMap.*;
import static frc.robot.RobotMap.AutoConstants.*;
import frc.robot.RobotMap.ClawConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ExtenderConstants;
import frc.robot.RobotMap.LifterConstants;
import frc.robot.RobotMap.LimelightConsants;
import frc.robot.RobotMap.LEDConstants;
import frc.robot.commands.LiftThrowCmd;
//import frc.robot.subsystems.Arduino;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Support;
import edu.wpi.first.wpilibj.smartdashboard.*;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private int countLoop;

    private RobotContainer m_robotContainer;
    private Lifter lift;
    private Gripper gripper;
    private Extender extender;
    // private Arduino arduino = new Arduino();
    private LEDSubsystem leds = new LEDSubsystem();
    private Support spt = new Support();

    private LiftThrowCmd lifterCommands;


    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    int autoMode = 2; //kTwoCubes;
    String shootMode = kCubeHigh;

    private final SendableChooser<String> aut_chooser = new SendableChooser<>();
    private final SendableChooser<String> sho_aut_chooser = new SendableChooser<>();

    private String shootChooser = "";

    int num = 0;
    int count = 0;
    private int timeDrivingAuto = 0;

    int countPressTime = 0;
    int countGripTime = 0;

    int liftCmdState = 0;

    boolean contnue = false;

    boolean balancing = false;

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        gripper = new Gripper();
        lift = new Lifter();
        extender = new Extender();

        lifterCommands =  new LiftThrowCmd(lift, gripper, extender);

        m_robotContainer.getSwerveSubsystem().setDriveMode(true);

        lift.init();
        gripper.init();
        extender.init();
        leds.init(gripper, lift, extender);

        leds.setCol(0, 255, 255, false);

        SmartDashboard.putNumber("autoMode", autoMode);

        sho_aut_chooser.setDefaultOption("Cube High", kCubeHigh);
        sho_aut_chooser.addOption("Cube Mid", kCubeMid);
        SmartDashboard.putData("shootMode", sho_aut_chooser);
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

        SmartDashboard.putNumber("Nav Pitch", IO.getPitch());
        SmartDashboard.putNumber("Nav Yaw", IO.getYaw(GyroType.kNAVX));
        SmartDashboard.putNumber("Nav Roll", IO.getRoll());
        // SmartDashboard.putNumber("Nav Pitch", IO.getPitch());


        // System.out.println(arduino.readArd());

        SmartDashboard.putNumber("Battery Voltage", (IO.getBattVoltage()));
        IO.getAprilTagValues();
        SmartDashboard.putBoolean("Battery Good (> 11.5)", (IO.getBattVoltage() > 11.5));
        m_robotContainer.getSwerveSubsystem().showData();

        leds.runLEDs();
    }

    @Override
    public void disabledInit() {
        leds.setCol(0, 255, 0,  false);
        //leds.setState(LEDConstants.STATE_LEDS_STATUS);
    }


    @Override
    public void disabledPeriodic() {
        IO.setLed(LimelightConsants.kLedOff);
        m_robotContainer.getSwerveSubsystem().setDriveMode(DriveConstants.kCoastMode);
        extender.showData();
        lift.setDriveMode(false);
        lift.showInfo();
        SmartDashboard.putBoolean("DIO_W", IO.getDIO(IO.SettingConstants.kWhite));
        SmartDashboard.putBoolean("DIO_Y", IO.getDIO(IO.SettingConstants.kYellow));

        /* ------------------------------ ROBOT BUTTONS ----------------------------- */
        if(IO.getDIO(IO.SettingConstants.kWhite)) {
            leds.setState(LEDConstants.STATE_LEDS_STATUS);
        } else if (IO.getDIO(IO.SettingConstants.kRed)) {
            // m_robotContainer.getSwerveSubsystem().setDriveMode(false);
        } else if (IO.getDIO(IO.SettingConstants.kYellow)) {
            SmartDashboard.putBoolean("DIO_W", IO.getDIO(IO.SettingConstants.kWhite));
            IO.zeroHeading();
        }

        /* ------------------------------ AUTO CHOOSER ------------------------------ */
        if(IO.getHIDButton(CMD_BUTTON_CU_HIGH)) {
            autoMode = kShootAndFarOutSpin;
        } else if(IO.getHIDButton(CMD_BUTTON_CU_MID)) {
            autoMode = kShootOutBalance;
        } else if (IO.getHIDButton(CMD_BUTTON_LOW)) {
            autoMode = kTwoCubes;
        }

        

        SmartDashboard.putNumber("autoMode", autoMode);
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

        leds.setCol((IO.isTeamRed() ? 255 : 0), 0, (IO.isTeamRed() ? 0 : 255), false);

        lift.setDriveMode(true);

        countGripTime = 0;
        liftCmdState = 0;

        lift.setStateLifter(LifterConstants.STATE_LIFT_UNKOWN);

        // m_robotContainer.getSwerveSubsystem().setDriveMode(DriveConstants.kBrakeMode);
        m_robotContainer.getSwerveSubsystem().setDriveMode(true);


        // lifterCommands = new LiftThrowCmd(lift, gripper, extender, 42, 5);

        // autoMode = (int) SmartDashboard.getNumber("autoMode", autoMode);
        shootMode = sho_aut_chooser.getSelected();

        SmartDashboard.putNumber("Selected Auto", autoMode);

        SmartDashboard.putBoolean("Loaded Path", false);

        if(autoMode == AutoConstants.kOut) {
            SmartDashboard.putBoolean("Loaded Path", true);
            m_autonomousCommand = m_robotContainer.runPath("Out", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
        }
        
        else if (autoMode == AutoConstants.kShootAndFarOutSpin || autoMode == AutoConstants.kTwoCubes) {
            SmartDashboard.putBoolean("Loaded Path", true);
            m_autonomousCommand = m_robotContainer.runPath("Out Far Spin", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
        }

        else if (autoMode == AutoConstants.kShootOutBalance) {
            SmartDashboard.putBoolean("Loaded Path", true);
            m_autonomousCommand = m_robotContainer.runPath("Bal", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);

        }
        
        else if (SmartDashboard.getNumber("autoMode", autoMode) == 5) {
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
        if (autoMode >= AutoConstants.kShootAndFarOutSpin && autoMode <= AutoConstants.kShootOutBalance) {
            switch(liftCmdState) {
                case UNKNOWN_HOME:
                    SmartDashboard.putBoolean("Auto Done?", false);
                    if(lift.currentlyHomed()){
                        liftCmdState = AUTO_STATE_LIFTER_DOWN;
                    }
                    lift.run();
                    break;
                
                case AUTO_STATE_LIFTER_DOWN:
                    // switch(shootMode) {
                    //     case kCubeHigh:
                    //         contnue = lift.autoRun(41, 5); // 41
                    //         break;
                    //     case kCubeMid:
                    //         contnue = lift.autoRun(68, 5); // 41
                    //         break;
                    // }
                    contnue = lift.autoRun(37, 4); // 41
                    if (contnue) {
                        liftCmdState = AUTO_STATE_SHOOT_HIGH;
                        contnue = false;
                    }
                    break;
            
                case AUTO_STATE_SHOOT_HIGH:
                    if (countGripTime < 50) {
                        gripper.setState(ClawConstants.CASE_EXPEL_CUBE_HIGH).run();
                        countGripTime++;
                    }
                    if (countGripTime >= 50) {
                        gripper.setState(ClawConstants.CASE_STOP).run();
                        liftCmdState = AUTO_STATE_HOME_LIFTER;
                        countGripTime = 0;
                    }
                    break;
                
                case AUTO_STATE_HOME_LIFTER:
                    contnue = lift.autoRun(5, 5);

                    if (contnue) {
                        liftCmdState = AUTO_STATE_MOVE_OUT;
                        contnue = false;
                    }
                    break;
                

                case AUTO_STATE_MOVE_OUT:
                    m_autonomousCommand.schedule();
                    liftCmdState = AUTO_STATE_MOVING_OUT;
                    break;
            
                case AUTO_STATE_MOVING_OUT:
                    timeDrivingAuto++;
                    SmartDashboard.getNumber("time dri", timeDrivingAuto);
                    if(timeDrivingAuto >= 170){
                        if(autoMode == kTwoCubes) {
                            m_autonomousCommand.cancel();
                            liftCmdState = AUTO_STATE_LIFTER_CUBE;
                            timeDrivingAuto = 0;

                        } else if(autoMode == kShootOutBalance){
                            m_autonomousCommand.cancel();
                            m_robotContainer.balance();
                        }
                    }
                    break;
        
                case AUTO_STATE_LIFTER_CUBE:
                    extender.setToDistance(0, 0.2);
                    contnue = lift.autoRun(LifterConstants.LIFTER_MAX_ROTATION - 10, 3);
                    if(contnue) {
                        countGripTime = 0;
                        contnue = false;
                        liftCmdState = AUTO_STATE_INTAKE_CUBE;
                    }
                    break;

                case AUTO_STATE_INTAKE_CUBE:
                    if (countGripTime < 50) {
                        gripper.setState(ClawConstants.CASE_INTAKE).run();
                        countGripTime++;
                    } else {
                        gripper.setState(ClawConstants.CASE_STOP).run();
                        contnue = lift.autoRun(0, 10);
                        if(contnue) {
                            liftCmdState = AUTO_STATE_RETURN_CUBE;
                            m_autonomousCommand = m_robotContainer.runPath("Path Back", AutoConstants.MAX_PATH_SPEED_AUTO, AutoConstants.MAX_PATH_ACCEL_AUTO);
                            m_autonomousCommand.schedule();
                            contnue = false;
                            countGripTime = 0;
                            break;
                        }
                    }
                    break;
                
                case AUTO_STATE_RETURN_CUBE:
                    
                    
                    // contnue = false;
                    // if(contnue) {
                    //     contnue = false;
                    //     liftCmdState = AUTO_STATE_RETURN_ZERO;
                    // }
                    break;
            
                case AUTO_STATE_RETURN_ZERO:
                    if(m_autonomousCommand.isFinished()) {
                        liftCmdState = AUTO_STATE_RETURN_LIFTER_DOWN;
                    }
                    break;
                
                case AUTO_STATE_RETURN_LIFTER_DOWN:
                    contnue = lift.autoRun(41, 5);
                    if(contnue) {
                        contnue = false;
                        liftCmdState = AUTO_STATE_RETURN_SHOOT;
                    }
                    break;

                case AUTO_STATE_RETURN_SHOOT:
                    if (countGripTime < 50) {
                        gripper.setState(ClawConstants.CASE_EXPEL_CUBE_HIGH).run();
                        countGripTime++;
                    }
                    if (countGripTime >= 50) {
                        gripper.setState(ClawConstants.CASE_STOP).run();
                        liftCmdState = AUTO_STATE_HOME_LIFTER;
                        countGripTime = 0;
                    }
        
                case AUTO_STATE_FINISH:
                    SmartDashboard.putBoolean("Auto Done?", true);
                    break;
            }
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until interrupted by another command, remove this line or comment it out.
        lift.init();

        leds.initLEDs();
        leds.setCol((IO.isTeamRed() ? 255 : 0), 0, (IO.isTeamRed() ? 0 : 255), false);
        
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
        if(!balancing){
            CommandScheduler.getInstance().run();

        }

        if(IO.isRightBumpPressed()) {
            leds.setPiece(LEDConstants.PIECE_TYPE_CONE);
        } else if(IO.isLeftBumpPressed()) {
            leds.setPiece(LEDConstants.PIECE_TYPE_CUBE);
        }

        lift.manualRun();
        gripper.manualControl();
        extender.runExtender();

        if (IO.getDriverButton(3)) { //68
            m_robotContainer.getSwerveSubsystem().setDriveMode(true);
        }
        
        if (IO.getDriverButton(4)) {
            m_robotContainer.getSwerveSubsystem().setDriveMode(false);
        }

        if (IO.isLeftAxisPressed() || IO.getHIDButton(CMD_BUTTON_HOME)){
            extender.setToDistance(0, 0.2);
            lift.setToDistance(0, 3);

        } else if(IO.isRightAxisPressed() || IO.getHIDButton(CMD_BUTTON_GROUND)) {
            extender.setToDistance(0, 0.2);
            lift.setToDistance(LifterConstants.LIFTER_MAX_ROTATION - 5, 5);

        } else if (IO.getHIDButton(CMD_BUTTON_CU_HIGH)) {
            extender.setToDistance(0, 0.2, 35);
            lift.setToDistance(41, 5);

        } else if (IO.isLeftBumpPressed() || IO.getHIDButton(CMD_BUTTON_CU_MID)) {
            extender.setToDistance(0, 0.2);
            lift.setToDistance(68, 5);

        } else if (IO.getHIDButton(CMD_BUTTON_DOUBLE)) {
            
            lift.setToDistance(30, 5);
            extender.setToDistance(10, 0.5, 35);
        } else if (IO.getHIDButton(CMD_BUTTON_CHUTE)) {
            extender.setToDistance(0, 0.2);
            lift.setToDistance(60, 3);

        } else if (IO.getHIDButton(CMD_BUTTON_LOW)) {
            extender.setToDistance(0, 0.2);
            lift.setToDistance(LifterConstants.LIFTER_MAX_ROTATION - 10, 3);
        } else if (IO.getHIDButton(CMD_BUTTON_CO_MID)) {
            extender.setToDistance(18, 2, 50);
            lift.setToDistance(47, 3);
        }

        if (IO.isBPressed() || IO.getHIDButton(CMD_BUTTON_STOP)) {
            leds.setCol((IO.isTeamRed() ? 255 : 0), 0, (IO.isTeamRed() ? 0 : 255), false);
        }


        if(IO.getDriverButton(10)) {
            IO.zeroHeading();
        }

        SmartDashboard.putNumber("MatchTime",DriverStation.getMatchTime());
        if(DriverStation.isTeleop() && DriverStation.getMatchTime() >=0 && DriverStation.getMatchTime() < 31) {
            leds.endGame(DriverStation.getMatchTime());
        }

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();


        IO.setLed(LimelightConsants.kPipelineLedSettings);
        lift.init();
        extender.init();

        leds.setCol((IO.isTeamRed() ? 255 : 0), 0, (IO.isTeamRed() ? 0 : 255), true);

        lift.setDriveMode(true);

        countGripTime = 0;
        liftCmdState = 0;

        // lift.setStateLifter(LifterConstants.STATE_LIFT_UNKOWN);

        // m_robotContainer.getSwerveSubsystem().setDriveMode(DriveConstants.kBrakeMode);
        m_robotContainer.getSwerveSubsystem().setDriveMode(true);


        // lifterCommands = new LiftThrowCmd(lift, gripper, extender, 42, 5);

        // autoMode = (int) SmartDashboard.getNumber("autoMode", autoMode);
        shootMode = sho_aut_chooser.getSelected();

        SmartDashboard.putNumber("Selected Auto", autoMode);

        SmartDashboard.putBoolean("Loaded Path", false);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        m_robotContainer.balance();
    }
}