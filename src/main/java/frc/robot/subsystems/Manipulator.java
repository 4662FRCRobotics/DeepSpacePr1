/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Manipulator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid m_grip;
  private DoubleSolenoid m_ballThrower;

  private boolean m_bIsGripOpen = true;

  private final int BALL_THROWER_FORWARD = 0;
  private final int BALL_THROWER_BACKWARD = 1;

  private final int MODULE_NUMBER = 1;
  private final int GRIP_OPEN = 2;
  private final int GRIP_CLOSE = 3;

  public Manipulator(){
    m_grip = new DoubleSolenoid(MODULE_NUMBER, GRIP_OPEN, GRIP_CLOSE);
    m_ballThrower = new DoubleSolenoid(MODULE_NUMBER, BALL_THROWER_FORWARD, BALL_THROWER_BACKWARD);
    // setGripForward();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean isGripOpen(){
    SmartDashboard.putBoolean("Grip solenoid value", m_bIsGripOpen);
    return m_bIsGripOpen;
  }

  public void setGripForward(){
    if(!m_bIsGripOpen) {
      m_grip.set(DoubleSolenoid.Value.kForward);
      m_bIsGripOpen = true;
    }
  }

  public void setGripBackward(){
    if(m_bIsGripOpen) {
      m_grip.set(DoubleSolenoid.Value.kReverse);
      m_bIsGripOpen = false;
    }  
  }

  public void setGripOff(){
    m_grip.set(DoubleSolenoid.Value.kOff);
  }

  public void PushBall(){
    m_ballThrower.set(DoubleSolenoid.Value.kReverse);
  }

  public void ResetPushBall(){
    m_ballThrower.set(DoubleSolenoid.Value.kForward);
  }

  public void StopPushBall(){
    m_ballThrower.set(DoubleSolenoid.Value.kOff);
  }
}
