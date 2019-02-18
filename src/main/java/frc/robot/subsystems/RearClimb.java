/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RearClimb extends Subsystem {

  private WPI_TalonSRX m_rearClimbMotor;

  private final double kCLIMB_SPEED = -0.5;

  public RearClimb(){
    m_rearClimbMotor = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("rearClimb"));
    m_rearClimbMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_rearClimbMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    
  }

  @Override
  public void initDefaultCommand() {
  }

  public void moveClimb(double speed){
    m_rearClimbMotor.set(speed);
  }

  public void extendClimb(){
    moveClimb(kCLIMB_SPEED);
  }

  public void retractClimb(){
    moveClimb(-kCLIMB_SPEED);
  }

  public void stopClimb(){
    moveClimb(0.0);
  }
}
