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

  private Joystick m_consoleBoard;

  private JoystickButton m_setParkLevel;
  private JoystickButton m_setBall1Level;
  private JoystickButton m_setBall2Level;
  private JoystickButton m_setBall3Level;
  private JoystickButton m_setPort1Level;
  private JoystickButton m_setPort2Level;
  private JoystickButton m_setPort3Level;

  private InternalButton m_setBrake;
  private InternalButton m_holdWristPosition;

  private final int VISION_LIGHT_BUTTON = 12;
  private final int UP_ELBOW = 5; 
  private final int DOWN_ELBOW = 3;   
  private final int UP_WRIST = 6;
  private final int DOWN_WRIST = 4;
  private final int TOGGLE_GRIP = 2;
  private final int PUSH_BALL = 1;
  private final int PARK_BTN = 1;
  private final int BALL1_BTN = 2;
  private final int BALL2_BTN = 3;
  private final int BALL3_BTN = 4;
  private final int PORT1_BTN = 5;
  private final int PORT2_BTN = 6;
  private final int PORT3_BTN = 7;
  private final String PARK = "park";
  private final String BALL1 = "ball1";
  private final String BALL2 = "ball2";
  private final String BALL3 = "ball3";
  private final String PORT1 = "port1";
  private final String PORT2 = "port2";
  private final String PORT3 = "port3";

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

    m_consoleBoard = new Joystick(2);

    m_setParkLevel = new JoystickButton(m_consoleBoard, PARK_BTN);
    m_setParkLevel.whenPressed(new SetArmLevel(PARK));

    m_setBall1Level = new JoystickButton(m_consoleBoard, BALL1_BTN);
    m_setBall1Level.whenPressed(new SetArmLevel(BALL1));

    m_setBall2Level = new JoystickButton(m_consoleBoard, BALL2_BTN);
    m_setBall2Level.whenPressed(new SetArmLevel(BALL2));

    m_setBall3Level = new JoystickButton(m_consoleBoard, BALL3_BTN);
    m_setBall3Level.whenPressed(new SetArmLevel(BALL3));

    m_setPort1Level = new JoystickButton(m_consoleBoard, PORT1_BTN);
    m_setPort1Level.whenPressed(new SetArmLevel(PORT1));

    m_setPort2Level = new JoystickButton(m_consoleBoard, PORT2_BTN);
    m_setPort2Level.whenPressed(new SetArmLevel(PORT2));
    
    m_setPort3Level = new JoystickButton(m_consoleBoard, PORT3_BTN);
    m_setPort3Level.whenPressed(new SetArmLevel(PORT3));

    m_setBrake = new InternalButton();
    m_setBrake.whenPressed(new SetBraken(0.1));

    m_holdWristPosition = new InternalButton();
    m_holdWristPosition.whenPressed(new HoldWristPosition());
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
  public void setHoldWristOn(){
    m_holdWristPosition.setPressed(true);
  }

  public void clearWristOn(){
    m_holdWristPosition.setPressed(false);
  }
}
