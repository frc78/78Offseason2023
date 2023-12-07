// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class FeedWheel extends SubsystemBase {
  private CANSparkMax wheelNeo;
  /** Creates a new FeedWheel. */
  public FeedWheel() {
    wheelNeo = new CANSparkMax(RobotConstants.WheelNeo, MotorType.kBrushless);
  }
  public void runFeedWheel(){
    wheelNeo.set(-.6);//Also subject to change during testing
  }

  public void stopFeedWheel() {
    wheelNeo.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}