/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Manipulator extends Subsystem {
  
  private DoubleSolenoid m_grip; 

  private final int GRIP_OPEN = 0;
  private final int GRIP_CLOSE = 1;

  public Manipulator(){
    m_grip = new DoubleSolenoid(GRIP_OPEN,GRIP_CLOSE);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setGripForward(){
    m_grip.set(DoubleSolenoid.Value.kForward);
  }
  public void setGripBackward(){
    m_grip.set(DoubleSolenoid.Value.kReverse);
  }
  public void setGripOff(){
    m_grip.set(DoubleSolenoid.Value.kOff);
  }
}
