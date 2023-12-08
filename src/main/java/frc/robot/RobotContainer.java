// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Systems.*;
import frc.robot.Systems.Chassis.*;
import frc.robot.Commands.*;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private Chassis m_chassis;
  private XboxController m_driveController, m_manipController;
  private final Shooter m_shooter;
  private final Feed m_feed;
  private final FeedWheel m_FeedWheel;
  private final FeedRun m_FeedRun;

  public RobotContainer() {
    m_chassis = new Chassis();

    m_driveController = new XboxController(0);
    m_manipController = new XboxController(1);

    m_shooter = new Shooter();
    m_feed = new Feed();
    m_FeedWheel = new FeedWheel();
    m_FeedRun = new FeedRun(m_feed);
    


  

    m_chassis.setDefaultCommand(new Drive(
      m_chassis,
      m_driveController::getLeftY,
      m_driveController::getLeftX,
      m_driveController::getRightX,
      m_driveController::getLeftTriggerAxis,
      m_driveController::getRightTriggerAxis,
      m_driveController::getYButton,
      m_driveController::getBButton,
      m_driveController::getAButton,
      m_driveController::getXButton
      ));
  }

  private void configureButtonBindings() {
    BooleanSupplier rightSupplier = new BooleanSupplier(){
      public boolean getAsBoolean(){
        return m_manipController.getRightTriggerAxis() > 0.5;
      }
    };
    new Trigger(rightSupplier).whileTrue(new SpinUp(m_shooter, RobotConstants.spinupVel, true));

    BooleanSupplier leftSupplier = new BooleanSupplier(){
      public boolean getAsBoolean(){
        return m_manipController.getLeftTriggerAxis() > 0.5;
      }
    };
    new Trigger(leftSupplier).whileTrue(new Fire(m_feed, m_FeedWheel));

    new Trigger(m_manipController :: getAButton).whileTrue(new FeedRun(m_feed));
      
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}