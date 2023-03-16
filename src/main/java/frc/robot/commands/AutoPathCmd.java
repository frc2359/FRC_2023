package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPathCmd extends SequentialCommandGroup {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    

    double pitchAngleDegrees    = gyro.getPitch();
    private final String kinematics = null;


    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                swerveSubsystem::getPose, // Pose supplier
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                swerveSubsystem::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerveSubsystem // Requires this drive subsystem
            )
        );
    }


    /**Balance the robot */
    public Command balance(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
                double xAxisRate = Math.sin(pitchAngleRadians) * -1;


                // 4. Construct desired chassis speeds
                ChassisSpeeds chassisSpeeds;
                chassisSpeeds = new ChassisSpeeds(xAxisRate, 0, 0);
                
            
                // 5. Convert chassis speeds to individual module states
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
                // 6. Output each module states to wheels
                swerveSubsystem.setModuleStates(moduleStates);
            }
        ));
        
    }
}
