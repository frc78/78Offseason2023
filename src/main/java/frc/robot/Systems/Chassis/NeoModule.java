// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Classes.ModuleConfig;
import frc.robot.Constants.RobotConstants;

/** Neo implementation of SwerveModule */
public class NeoModule implements SwerveModule {
    protected ModuleConfig config;

    protected CANSparkMax drive;
    protected CANSparkMax steer;
    protected CANCoder enc;

    protected SparkMaxPIDController drivePID;
    protected SparkMaxPIDController steerPID;
    private RelativeEncoder driveEnc;
    private AbsoluteEncoder steerEnc;
    
    NeoModule(ModuleConfig config) {
        this.config = config;
        // drive = new Spark(this.config.driveID);
        // steer = new Spark(this.config.steerID);

        drive = new CANSparkMax(this.config.driveID, MotorType.kBrushless);
        steer = new CANSparkMax(this.config.steerID, MotorType.kBrushless);
        driveEnc = drive.getEncoder();
        steerEnc = steer.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        drivePID = drive.getPIDController();
        steerPID = steer.getPIDController();
        enc = new CANCoder(this.config.encID);
    }   

    @Override
    public void initialize() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drive.restoreFactoryDefaults();
    steer.restoreFactoryDefaults();
    drivePID.setFeedbackDevice(driveEnc);
    steerPID.setFeedbackDevice(steerEnc);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEnc.setPositionConversionFactor(RobotConstants.DRIVE_TO_METERS);
    driveEnc.setVelocityConversionFactor(RobotConstants.DRIVE_VEL_TO_METERS);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    steerEnc.setPositionConversionFactor(RobotConstants.STEER_TO_METERS);
    steerEnc.setVelocityConversionFactor(RobotConstants.STEER_VEL_TO_METERS);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    steerEnc.setInverted(RobotConstants.STEER_ENC_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    steerPID.setPositionPIDWrappingEnabled(true);
    steerPID.setPositionPIDWrappingMinInput(RobotConstants.STEER_ENC_MIN);
    steerPID.setPositionPIDWrappingMaxInput(RobotConstants.STEER_ENC_MAX);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivePID.setP(RobotConstants.K_DRIVE_P);
    drivePID.setI(RobotConstants.K_DRIVE_I);
    drivePID.setD(RobotConstants.K_DRIVE_D);
    drivePID.setFF(RobotConstants.K_DRIVE_FF);
    drivePID.setOutputRange(RobotConstants.DRIVE_OUT_MIN,
        RobotConstants.DRIVE_OUT_MAX);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    steerPID.setP(RobotConstants.K_STEER_P);
    steerPID.setI(RobotConstants.K_STEER_I);
    steerPID.setD(RobotConstants.K_STEER_D);
    steerPID.setFF(RobotConstants.K_STEER_FF);
    steerPID.setOutputRange(RobotConstants.STEER_OUT_MIN,
        RobotConstants.STEER_OUT_MAX);

    drive.setIdleMode(RobotConstants.DRIVE_IDLE);
    steer.setIdleMode(RobotConstants.STEER_IDLE);
    // drive.setSmartCurrentLimit(RobotConstants.kDrivingMotorCurrentLimit); // TODO maybe set this, not required probably
    // steer.setSmartCurrentLimit(RobotConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drive.burnFlash();
    steer.burnFlash();

    // m_chassisAngularOffset = chassisAngularOffset; // TODO what is this for?
    // m_desiredState.angle = new Rotation2d(steerEnc.getPosition());
    driveEnc.setPosition(0);
    }

    @Override
    public void resetToAbsolute() {
        // TODO Auto-generated method stub
    }

    @Override
    public double getVelocity() {
        return drive.get();
    }

    @Override
    public Rotation2d getRelEncoderPosition() {
        // TODO Check if this right
        return Rotation2d.fromDegrees(enc.getPosition());
    }

    @Override
    public Rotation2d getAbsEncoderPosition() {
        // return steer.getAbsoluteEncoder(Type.kHallSensor); //TODO
        return null;
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return null;
    }

    @Override
    public void setVelocity(double velocity) {
        drive.set(velocity);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        steerPID.setReference(rotation.getRotations(),  CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setState(SwerveModuleState state) {
        setRotation(state.angle);
        setVelocity(state.speedMetersPerSecond);
    }
}
