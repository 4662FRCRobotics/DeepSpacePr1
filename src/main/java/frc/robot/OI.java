/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);  We lost the game.

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  // Vamos

  // 1 : trigger
  // 2 : thumb button
  // 3 - 6 : top
  // 7 - 12 : base

  private Joystick m_driveStick;

  private JoystickButton m_visionLightButton;

  private JoystickButton m_moveElbowUpButton;
  private JoystickButton m_moveElbowDownButton;
  private JoystickButton m_moveWristDownButton;
  private JoystickButton m_moveWristUpButton;

  private JoystickButton m_openGripButton;
  private JoystickButton m_closeGripButton;

  private final int VISION_LIGHT_BUTTON = 12;
  private final int UP_ELBOW = 5; 
  private final int DOWN_ELBOW = 3;   
  private final int UP_WRIST = 6;
  private final int DOWN_WRIST = 4;
  private final int OPEN_GRIP = 1;
  private final int CLOSE_GRIP = 2;

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

    m_openGripButton = new JoystickButton(m_driveStick, OPEN_GRIP);
    m_openGripButton.whileHeld(new GripOpen());

    m_closeGripButton = new JoystickButton(m_driveStick, CLOSE_GRIP);
    m_closeGripButton.whileHeld(new GripClose());
  }

  // velocity * 2 / (throttle + 3)
  public double getVelocity(){
    return m_driveStick.getY() * 2 / (m_driveStick.getThrottle() + 3);
  }

  public double getHeading(){
    
    return m_driveStick.getTwist() * 2 / (m_driveStick.getThrottle() + 3);
  }
}
