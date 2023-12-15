// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systems.Feed;
//import frc.robot.Systems.Indexer;
//import frc.robot.Systems.Intake;
import frc.robot.Systems.FeedWheel;

public class Fire extends Command {
  private Feed m_feed;
  private FeedWheel m_feedWheel;
  /** Creates a new RunFeed. */
  public Fire(Feed subsystem, FeedWheel subsystem3) {
    m_feed = subsystem;
    m_feedWheel = subsystem3;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feed, m_feedWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feed.feedRun(.60);
    m_feedWheel.runFeedWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.stopFeed();
    m_feedWheel.stopFeedWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}