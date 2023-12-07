// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed extends SubsystemBase {
  public CANSparkMax beltneo;
  public DigitalInput input;
  
  /** Creates a new Feed. */
  public Feed() {
   beltneo = new CANSparkMax(RobotConstants.BeltNeo, MotorType.kBrushless);
   input = new DigitalInput(0);
   
  }
  
  public void feedRun(double m_feedSpeed) {
    
      beltneo.set(m_feedSpeed);//subject to change during testing
     
  }
  public void feedRunIfNoCargo(double m_feedSpeed) {
    if (input.get()){
      beltneo.set(0.0);
    }
    else{
      beltneo.set(m_feedSpeed);//subject to change during testing
    }
     
  }
  
  public void feedReverse() {
    beltneo.set(-.50);
  }

  public void stopFeed() {
    beltneo.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}