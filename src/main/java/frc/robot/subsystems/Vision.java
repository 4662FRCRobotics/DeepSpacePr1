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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Spark m_ledController;
  private boolean m_bIsLightOn;
  private NetworkTable m_VisionTable;
  private boolean m_bIsVisionOn;
  private NetworkTableEntry vTargetOffset;

  private NetworkTableEntry vIsVisionOn;
  private NetworkTableEntry vIsTargetFound;

  public Vision() {

    m_ledController = new Spark(Robot.m_robotMap.getPortNumber("ledController"));
    m_bIsLightOn = false;
    m_VisionTable = NetworkTableInstance.getDefault().getTable("Vision");
    m_bIsVisionOn = false;

    vIsVisionOn = m_VisionTable.getEntry("isVisionOn");
    vIsVisionOn.setBoolean(m_bIsVisionOn);

    vTargetOffset = m_VisionTable.getEntry("targetOffset");

    vIsTargetFound = m_VisionTable.getEntry("isTargetFound");

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }

  private void updateVisionTable() {
    vIsVisionOn.setBoolean(m_bIsVisionOn);
  }

  public boolean IsVisionOn() {
    return m_bIsVisionOn;
  }

public boolean IsTargetFound() {
  return vIsTargetFound.getBoolean(false);
}

  public void toggleVision() {
    if (m_bIsVisionOn) {
      m_ledController.set(0);
      
    } else {
      m_ledController.set(1);
    }
    m_bIsLightOn = !m_bIsLightOn;
    m_bIsVisionOn = !m_bIsVisionOn;
    updateVisionTable();
  }

  public void turnVisionOff() {
    m_ledController.set(0);
    m_bIsLightOn = false;
    m_bIsVisionOn = false;
    updateVisionTable();
  }

  public void turnVisionOn() {
    m_ledController.set(1);
    m_bIsLightOn = true;
    m_bIsVisionOn = true;
    updateVisionTable();
  }

  public double getTargetCenter() {
    double dTargetCenter = vTargetOffset.getDouble(0); 
    return dTargetCenter;
  }
}
