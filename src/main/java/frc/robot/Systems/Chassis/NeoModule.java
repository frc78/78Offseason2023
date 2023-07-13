// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Classes.ModuleConfig;

/** Add your docs here. */
public class NeoModule implements SwerveModule {
    protected ModuleConfig config;
    // protected Spark drive;
    // protected Spark steer;

    protected CANSparkMax drive;
    protected CANSparkMax steer;
    protected CANCoder enc;

    protected SparkMaxPIDController steerPID;
    private RelativeEncoder steer_encoder;
    public double steer_P, steer_I, steer_D, steer_Iz, steer_FF, steer_MaxOutput, steer_MinOutput;

    
    NeoModule(ModuleConfig config) {
        this.config = config;
        // drive = new Spark(this.config.driveID);
        // steer = new Spark(this.config.steerID);

        drive = new CANSparkMax(this.config.driveID, MotorType.kBrushless);
        steer = new CANSparkMax(this.config.steerID, MotorType.kBrushless);
        enc = new CANCoder(this.config.encID);

    }   

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    steerPID = steer.getPIDController();

    // Encoder object created to display position values
    steer_encoder = steer.getEncoder();

    // PID coefficients
    steer_P = 0.1; 
    steer_I = 1e-4;
    steer_D = 1; 
    steer_Iz = 0; 
    steer_FF = 0; 
    steer_MaxOutput = 1; 
    steer_MinOutput = -1;

    // set PID coefficients
    steerPID.setP(steer_P);
    steerPID.setI(steer_I);
    steerPID.setD(steer_D);
    steerPID.setIZone(steer_Iz);
    steerPID.setFF(steer_FF);
    steerPID.setOutputRange(steer_MinOutput, steer_MaxOutput);

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
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public SwerveModulePosition getPosition() {
        // TODO Auto-generated method stub
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
        // TODO Auto-generated method stub
        
    }
}
