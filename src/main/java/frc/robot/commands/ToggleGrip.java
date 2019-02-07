/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class ToggleGrip extends TimedCommand {

  private boolean m_bisGripOpen;

  public ToggleGrip(double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(timeout);
    requires(Robot.m_manipulator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_bisGripOpen = Robot.m_manipulator.isGripOpen();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_bisGripOpen){
      Robot.m_manipulator.setGripBackward();
    }
    else{
      Robot.m_manipulator.setGripForward();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_manipulator.setGripOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
