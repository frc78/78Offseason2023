// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SpinUp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Systems.Shooter;
import frc.robot.RobotConstants;

public class SpinUp extends CommandBase {
  private Shooter shooter;
  private double vel;
  private boolean isHood;


  /** Creates a new Shoot. */
  public SpinUp(Shooter shooter, double Velocity, boolean isHood) {
    this.shooter = shooter;
    this.vel = Velocity;
    this.isHood = isHood;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting spin up!");
    this.shooter.isHood(isHood);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.startWheel(vel);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopWheely();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}