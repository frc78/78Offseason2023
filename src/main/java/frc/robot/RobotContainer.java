// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Systems.*;
import frc.robot.Systems.Chassis.*;
import frc.robot.Commands.*;

public class RobotContainer {
  private Chassis m_chassis;
  private XboxController m_drive_controller;

  public RobotContainer() {
    m_chassis = new Chassis();

    m_chassis.setDefaultCommand(new Drive(m_chassis, m_drive_controller));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}