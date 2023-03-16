package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoPathCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -IO.getDriveY(),
                () -> -IO.getDriveX(),
                () -> -IO.getDriveTwist(),
                () -> false));

        //configureButtonBindings();
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    /**Runs inputted path from the helper application "Path Planner (from Microsoft Store)
     * @param pathName is the name of the path set in PathPlanner
     * @param maxV is the velocity on the path
     * @param maxAccel is the acceleration of the path
     * @return the command for the path the follow */
    public Command runPath(String pathName, int maxV, int maxAccel) {
        final AutoPathCmd command = new AutoPathCmd();
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(pathName, new PathConstraints(maxV, maxAccel));
        return command.followTrajectoryCommand(swerveSubsystem, examplePath, true);
    }

    /**Runs inputted path (that includes events) from the helper application "Path Planner (from Microsoft Store)
     * @param pathName is the name of the path set in PathPlanner
     * @param maxV is the velocity on the path
     * @param maxAccel is the acceleration of the path
     * @param events is a Hash Map of events that are to be run (in the format ("Key", event()))
     * @return the command for the path the follow */
    public Command runPathWithEvents(String pathName, int maxV, int maxAccel, HashMap<String, Command> events) {
        // final AutoPathCmd command = new AutoPathCmd();

        PathPlannerTrajectory examplePath = PathPlanner.loadPath(pathName, new PathConstraints(maxV, maxAccel));
        
        FollowPathWithEvents command = new FollowPathWithEvents(
                runPath(pathName, maxV, maxAccel), examplePath.getMarkers(), events
                );

        return command;
        // return command.followTrajectoryCommand(swerveSubsystem, examplePath, true);
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
