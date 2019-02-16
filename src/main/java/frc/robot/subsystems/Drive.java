/*----------------------------------------------------------------------------*/
/* Copyright (C) 2018 FIRST. All Rights Reserved.                             */
/* open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.ArcadeDrive;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


 // I did just lost the game?
/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //The New World Order is wacthing you.

  private CANSparkMax m_leftController1;
  private CANSparkMax m_leftController2;
  private CANSparkMax m_rightController1;
  private CANSparkMax m_rightController2;

  private SpeedControllerGroup m_leftControlGroup;
  private SpeedControllerGroup m_rightControlGroup;

  private DifferentialDrive m_robotDrive;

  private CANEncoder m_leftEncoder1;
  private CANEncoder m_rightEncoder1;

  private final double kRAMP_RATE = 0.5;
  private final double kENCODER_PULSES_PER_REV = 1;
  private final double kGEARBOX_REDUCTION = (50/12) * (60/14);
  private final double kTIRE_SIZE = 7.9; 

  public Drive(){

    m_leftController1 = new CANSparkMax(Robot.m_robotMap.getPortNumber("leftController1"), MotorType.kBrushless);
    m_leftController2 = new CANSparkMax(Robot.m_robotMap.getPortNumber("leftController2"), MotorType.kBrushless);
    m_rightController1 = new CANSparkMax(Robot.m_robotMap.getPortNumber("rightController1"), MotorType.kBrushless);
    m_rightController2 = new CANSparkMax(Robot.m_robotMap.getPortNumber("rightController2"), MotorType.kBrushless);

    m_leftController1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_leftController2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightController1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightController2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_leftController1.setOpenLoopRampRate(kRAMP_RATE);
    m_leftController2.setOpenLoopRampRate(kRAMP_RATE);
    m_rightController1.setOpenLoopRampRate(kRAMP_RATE);
    m_rightController2.setOpenLoopRampRate(kRAMP_RATE);

    m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);

    m_leftControlGroup.setInverted(false);
    m_rightControlGroup.setInverted(false);

    m_robotDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);

    m_leftEncoder1 = m_leftController1.getEncoder();
    m_rightEncoder1 = m_rightController1.getEncoder();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDrive());
    
  }

  public void arcadeDrive(double velocity, double heading){
    double dDriveInvert = -1;
    m_robotDrive.arcadeDrive(velocity * dDriveInvert, heading);
    smartDashBoardDisplay();
  }

  private void smartDashBoardDisplay(){
    double distancePerEncoderTic =  kGEARBOX_REDUCTION / (kTIRE_SIZE * Math.PI) / 12;
    SmartDashboard.putNumber("leftencoder", m_leftEncoder1.getPosition() / distancePerEncoderTic);
    SmartDashboard.putNumber("rightencoder", m_rightEncoder1.getPosition() / distancePerEncoderTic);
  }
}
