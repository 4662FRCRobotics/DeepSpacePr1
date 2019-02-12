/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  // Vamos

  // 1 : trigger
  // 2 : thumb button
  // 3 - 6 : top
  // 7 - 12 : base

  private Joystick m_driveStick;

  private JoystickButton m_visionLightButton;
  private JoystickButton m_moveElbowUpButton;
  private JoystickButton m_moveElbowDownButton;
  private JoystickButton m_moveWristUpButton;
  private JoystickButton m_moveWristDownButton;
  private JoystickButton m_toggleGripButton;
  private JoystickButton m_pushBall;

  private InternalButton m_setBrake;

  private final int VISION_LIGHT_BUTTON = 12;
  private final int UP_ELBOW = 5; 
  private final int DOWN_ELBOW = 3;   
  private final int UP_WRIST = 6;
  private final int DOWN_WRIST = 4;
  private final int TOGGLE_GRIP = 2;
  private final int PUSH_BALL = 1;

  private final double GRIP_TIME = 0.2;

  // I just lost the game

  public OI() {
    m_driveStick = new Joystick(0);

    m_visionLightButton = new JoystickButton(m_driveStick, VISION_LIGHT_BUTTON);
    m_visionLightButton.whenPressed(new ToggleVisionLight());

    m_moveElbowUpButton = new JoystickButton(m_driveStick, UP_ELBOW);
    m_moveElbowUpButton.whileHeld(new MoveElbowUp());

    m_moveElbowDownButton = new JoystickButton(m_driveStick, DOWN_ELBOW);
    m_moveElbowDownButton.whileHeld(new MoveElbowDown());

    m_moveWristUpButton = new JoystickButton(m_driveStick, UP_WRIST);
    m_moveWristUpButton.whileHeld(new MoveWristUp());

    m_moveWristDownButton = new JoystickButton(m_driveStick, DOWN_WRIST);
    m_moveWristDownButton.whileHeld(new MoveWristDown());

    m_toggleGripButton = new JoystickButton(m_driveStick, TOGGLE_GRIP);
    m_toggleGripButton.whenPressed(new ToggleGrip(GRIP_TIME));

    m_pushBall = new JoystickButton(m_driveStick, PUSH_BALL);
    m_pushBall.whenPressed(new LaunchBall());

    m_setBrake = new InternalButton();
    m_setBrake.whenPressed(new SetBraken(0.1));
  }

  // velocity * 2 / (throttle + 3)
  public double getVelocity(){
    return m_driveStick.getY() * 2 / (m_driveStick.getThrottle() + 3);
  }

  public double getHeading(){
    
    return m_driveStick.getTwist() * 2 / (m_driveStick.getThrottle() + 3);
  }

  public void setBrakeOn(){
    m_setBrake.setPressed(true);
  }

  public void clearBrakeOn(){
    m_setBrake.setPressed(false);
  }
}
