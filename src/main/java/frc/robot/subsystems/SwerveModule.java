package frc.robot.subsystems;

import static frc.robot.RobotMap.*;

import java.lang.Math;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {

    private final TalonFX driveMotor;               // Use Falcon500 for drive
    private final CANSparkMax turningMotor;         // Use NEO for turning

    //private final CANEncoder driveEncoder;        // Use encoder in Falcon500 for drive
    private final RelativeEncoder turningEncoder;   // Use SparkMax encoder for turning

    private final PIDController turningPidController;
    private final PIDController drivingPidController;

    private final CANCoder absoluteEncoder;         // Use CANCoder for absolute position
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId, "rio");
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        //driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        drivingPidController = new PIDController(ModuleConstants.kPDriving, 0, 0);
        drivingPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    /**Set whether the drive motor is inverted 
     * @param inv is true when you want to set it to inverted
    */
    public void invertDrive(boolean inv) {
        driveMotor.setInverted(inv);
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderDeg() {
        return absoluteEncoder.getAbsolutePosition();   // No offset
    }

    public double getAbsoluteEncoderRad() {
        double angle = Math.toRadians(getAbsoluteEncoderDeg()) - absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            // could update driveMotor to velocity control
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput,0);
        turningMotor.set(0);
    }
}