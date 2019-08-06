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

  // Joystick
  // 1 : trigger
  // 2 : thumb button
  // 3 - 6 : top
  // 7 - 12 : base

  /* Game Pad:
    X (1)
    A (2)
    B (3)
    Y (4)
    
    LB (5): Wrist Down
    RB (6): Wrist Up
    LT (7): ELbow Down
    RT (8): Elbow Up
    
    Back (9)
    Start (10)
    
    Left Joystick Button (11)
    Right Joystick Button (12)
  */

  private Joystick m_driveStick;

  private JoystickButton m_visionLightButton;
 

  private Joystick m_operatorPad;

  private JoystickButton m_moveElbowUpButton;
  private JoystickButton m_moveElbowDownButton;
  private JoystickButton m_moveWristUpButton;
  private JoystickButton m_moveWristDownButton;
  private JoystickButton m_extendRearClimb;
  private JoystickButton m_retractRearClimb;
  private JoystickButton m_toggleGripButton;
  private JoystickButton m_pushBall;
  private JoystickButton m_intakeCargo;


  private Joystick m_consoleBoard;

  private JoystickButton m_setParkLevel;
  private JoystickButton m_setBall1Level;
  private JoystickButton m_setBall2Level;
  private JoystickButton m_setBall3Level;
  private JoystickButton m_setPort1Level;
  private JoystickButton m_setPort2Level;
  private JoystickButton m_setPort3Level;
  private JoystickButton m_setBallCSLevel;
  private JoystickButton m_setPortCSLevel;

  private InternalButton m_setBrake;
  private InternalButton m_holdWristPosition;
  private InternalButton m_gripClose;

  // operator/game pad button ports
  private final int VISION_LIGHT_BUTTON = 12;
  private final int UP_ELBOW = 8; 
  private final int DOWN_ELBOW = 7;   
  private final int UP_WRIST = 6;
  private final int DOWN_WRIST = 5;
  private final int EXTEND_REAR_CLIMB = 9;
  private final int RETRACT_REAR_CLIMB = 10;
  private final int TOGGLE_GRIP = 1;
  private final int PUSH_BALL = 4;
  private final int INTAKE_CARGO = 2;

  // console board button ports
  private final int PARK_BTN = 1;
  private final int BALL1_BTN = 2;
  private final int BALL2_BTN = 3;
  private final int BALL3_BTN = 4;
  private final int PORT1_BTN = 5;
  private final int PORT2_BTN = 6;
  private final int PORT3_BTN = 7;
  private final int BALLCS_BTN =8;
  private final int PORTCS_BTN = 9;

  private final double GRIP_TIME = 0.2;

  // I just lost the game

  public OI() {
     //Joystick
    m_driveStick = new Joystick(0);

    m_visionLightButton = new JoystickButton(m_driveStick, VISION_LIGHT_BUTTON);
    m_visionLightButton.whenPressed(new ToggleVisionLight());


     //Game Pad
    m_operatorPad = new Joystick(1);

    m_moveElbowUpButton = new JoystickButton(m_operatorPad, UP_ELBOW);
    m_moveElbowUpButton.whileHeld(new MoveElbowUp());

    m_moveElbowDownButton = new JoystickButton(m_operatorPad, DOWN_ELBOW);
    m_moveElbowDownButton.whileHeld(new MoveElbowDown());

    m_moveWristUpButton = new JoystickButton(m_operatorPad, UP_WRIST);
    m_moveWristUpButton.whileHeld(new MoveWristUp());

    m_moveWristDownButton = new JoystickButton(m_operatorPad, DOWN_WRIST);
    m_moveWristDownButton.whileHeld(new MoveWristDown());

    m_intakeCargo = new JoystickButton(m_operatorPad, INTAKE_CARGO);
    m_intakeCargo.whileHeld(new IntakeCargo());


    //gears stripped on version 1
    m_extendRearClimb = new JoystickButton(m_operatorPad, EXTEND_REAR_CLIMB);
    m_extendRearClimb.whileHeld(new AutoRearClimbExtend(0.1));
    //Don't be a Bully!
    m_retractRearClimb = new JoystickButton(m_operatorPad, RETRACT_REAR_CLIMB);
    m_retractRearClimb.whileHeld(new AutoRearClimbRetract(0.1));

    m_toggleGripButton = new JoystickButton(m_operatorPad, TOGGLE_GRIP);
    m_toggleGripButton.whenPressed(new ToggleGrip(GRIP_TIME));

    m_pushBall = new JoystickButton(m_operatorPad, PUSH_BALL);
    m_pushBall.whenPressed(new LaunchBall());
    
    //Arduino
    m_consoleBoard = new Joystick(2);

    m_setParkLevel = new JoystickButton(m_consoleBoard, PARK_BTN);
    m_setParkLevel.whenPressed(new SetArmLevel(ArmSetPoint.PARK));

    m_setBall1Level = new JoystickButton(m_consoleBoard, BALL1_BTN);
    m_setBall1Level.whenPressed(new SetArmLevel(ArmSetPoint.BALL1));

    m_setBall2Level = new JoystickButton(m_consoleBoard, BALL2_BTN);
    m_setBall2Level.whenPressed(new SetArmLevel(ArmSetPoint.BALL2));

    m_setBall3Level = new JoystickButton(m_consoleBoard, BALL3_BTN);
    m_setBall3Level.whenPressed(new SetArmLevel(ArmSetPoint.BALL3));

    m_setPort1Level = new JoystickButton(m_consoleBoard, PORT1_BTN);
    m_setPort1Level.whenPressed(new SetArmLevel(ArmSetPoint.PORT1));

    m_setPort2Level = new JoystickButton(m_consoleBoard, PORT2_BTN);
    m_setPort2Level.whenPressed(new SetArmLevel(ArmSetPoint.PORT2));
    
    m_setPort3Level = new JoystickButton(m_consoleBoard, PORT3_BTN);
    m_setPort3Level.whenPressed(new SetArmLevel(ArmSetPoint.PORT3));

    m_setBallCSLevel = new JoystickButton(m_consoleBoard, BALLCS_BTN);
    m_setBallCSLevel.whenPressed(new SetArmLevel(ArmSetPoint.BALLCS));

    m_setPortCSLevel = new JoystickButton(m_consoleBoard, PORTCS_BTN);
    m_setPortCSLevel.whenPressed(new SetArmLevel(ArmSetPoint.PORTCS));

    //Internal Buttons
    m_setBrake = new InternalButton();
    m_setBrake.whenPressed(new SetBraken(0.1));

    m_holdWristPosition = new InternalButton();
    m_holdWristPosition.whenPressed(new HoldWristPosition());

    m_gripClose = new InternalButton();
    m_gripClose.whenPressed(new GripClose());
  }

  // velocity * 2 / (throttle + 3)
  public double getVelocity(){
    return m_driveStick.getY() * 2 / (m_driveStick.getThrottle() + 3);
  }

  public double getHeading(){
    
    return m_driveStick.getTwist() * 2 / (m_driveStick.getThrottle() + 3);
  }

  public double getWristJoystickY(){
    return -m_operatorPad.getY();
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

  public void setGripClose(){
    m_gripClose.setPressed(true);
  }

  public void clearGripClose(){
    m_gripClose.setPressed(false);
  }

  public String getAutoStartPos(){
    int startPosNumber = m_consoleBoard.getPOV(0);
    String startPosValue = "teleop";
    switch(startPosNumber){
      case 45:
        startPosValue = "leftHab2";
        break;
      case 90:
        startPosValue = "rightHab2";
        break;
      default:
        startPosValue = "teleop";
    }
    return startPosValue;
  }
}


