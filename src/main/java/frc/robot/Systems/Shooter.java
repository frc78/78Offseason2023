// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {
  
  private TalonFX flywheelMain;
  private TalonFX backFlywheel;
  private TalonFX feed;

  private PIDController flywheelPID;

  public Shooter() {
    flywheel = new TalonFX(RobotConstants.FLYWHEEL_L_ID)
  }

  public void setFlywheel(double speed) {
     
  }
}
