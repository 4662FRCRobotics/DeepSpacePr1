/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ManipulatorIntake;

public class IntakeCargo extends Command {
  
  private final double INTAKE_MOTOR_SPEED = -0.4;
  private final double INTAKE_MOTOR_OFF = 0;

  public IntakeCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_manipulatorIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_manipulatorIntake.setMotorSpeed(INTAKE_MOTOR_SPEED);
    Robot.m_oi.setGripClose();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_manipulatorIntake.setMotorSpeed(INTAKE_MOTOR_OFF);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
