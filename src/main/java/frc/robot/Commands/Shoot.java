// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Systems.Shooter;

public class Shoot extends CommandBase {

  private Shooter shooter;

  private double deltaX, deltaY;
  private double vel;


  public Shoot(double deltaX, double deltaY, Shooter shooter) {
    this.deltaX = deltaX;
    this.deltaY = deltaY;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vel = calcVel(deltaX, deltaY, RobotConstants.HOOD_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Calculates the velocity using parabolic model and then adjusted with test values. Andrew's baby */
  double calcVel(double deltaX, double deltaY, double exitTheta) {
    double g = -9.81;
    double num = g * Math.pow(deltaX, 2);
    double den = 2 * Math.pow(Math.cos(exitTheta), 2) * (deltaX * Math.tan(exitTheta) - deltaY);
    double calc = num / den;
    return calc /*times constants, TODO */;
  }
}