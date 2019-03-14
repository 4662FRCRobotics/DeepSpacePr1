/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveElbowSync extends Command {

  private double m_dElbowSpeed;
  private final double kELBOW_DOWN_SPEED = 0.6;
  private final double kELBOW_UP_SPEED = -0.8;
  public MoveElbowSync(boolean isElbowDirUp) {
    requires(Robot.m_elbowJoint);
    if (isElbowDirUp) {
      m_dElbowSpeed = kELBOW_UP_SPEED;
    } else {
      m_dElbowSpeed = kELBOW_DOWN_SPEED;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_elbowJoint.isControlAuto()) {
      // start move write to setpoint based on degrees
    }
    Robot.m_elbowJoint.moveJointMotor(m_dElbowSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elbowJoint.moveJointMotor(0.0);
    Robot.m_oi.setStopWristSync();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
