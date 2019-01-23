/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 * ARM
 * wRist
 * Mechanism
 */
public class ARMJoint extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private final String MOTORNUMBERONE = "1";
  private final String MOTORNUMBERTWO = "2";

  private WPI_TalonSRX m_jointMotor1;
  private WPI_TalonSRX m_jointMotor2;

  private SpeedControllerGroup m_jointMotorGroup;

  public ARMJoint(String motorString){
    this(1, motorString);
  }

  public ARMJoint(int motorCount, String motorString){

    if (motorCount > 2){
      motorCount = 2;
    }
    else if (motorCount < 1){
      motorCount = 1;
    }

    m_jointMotor1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber(motorString + MOTORNUMBERONE));
    m_jointMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_jointMotor1.setSensorPhase(false);

    switch (motorCount){
      case 1:
        m_jointMotorGroup = new SpeedControllerGroup(m_jointMotor1);
        break;
      
      case 2:
        m_jointMotor2 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber(motorString + MOTORNUMBERTWO));
        m_jointMotorGroup = new SpeedControllerGroup(m_jointMotor1, m_jointMotor2);
        
        break;
    }

    m_jointMotorGroup.setInverted(true);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
