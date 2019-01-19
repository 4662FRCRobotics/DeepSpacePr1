/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Spark m_ledController;
  private boolean m_bLightOn;

  public Vision(){
    m_ledController = new Spark(Robot.m_robotMap.getPortNumber("ledController"));
    m_bLightOn = false;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
  }

  public boolean IsLightOb() {
    return m_bLightOn;
  }

  public void toggleLight() {
    if (m_bLightOn){
      m_ledController.set(0);
    } else {
      m_ledController.set(-1);
    }
    m_bLightOn = !m_bLightOn;
  }

  public void turnLightOff() {
    m_ledController.set(0);
    m_bLightOn = false;
  }
}
