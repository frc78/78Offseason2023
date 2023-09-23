// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {
  
  private TalonFX flywheelMain;
  private TalonFX flywheelFollow;
  private TalonFX backFlywheel;
  private TalonFX feed;

  private PIDController velPID;
  private SimpleMotorFeedforward feedforward;

  public Shooter() {
    flywheelMain = new TalonFX(RobotConstants.FLYWHEEL_M_ID);
    flywheelFollow = new TalonFX(RobotConstants.FLYWHEEL_F_ID);
    flywheelFollow.follow(flywheelMain);
    flywheelFollow.setInverted(true); //TODO MUST CHECK IF THIS WILL INVERT CORRECTLY

    velPID.setPID(RobotConstants.FLYWHEEL_P, RobotConstants.FLYWHEEL_I, RobotConstants.FLYWHEEL_D);
    feedforward = new SimpleMotorFeedforward(RobotConstants.FLYWHEEL_KS, RobotConstants.FLYWHEEL_KV, RobotConstants.FLYWHEEL_KA);
  }

  @Override
  public void periodic () {
    
    flywheelMain.set(ControlMode.Velocity, velPID.calculate(flywheelMain.getSelectedSensorVelocity()));
  }

  public void setFlywheel(double speed) {
     
  }
}
