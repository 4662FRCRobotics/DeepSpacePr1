/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.ArcadeDrive;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX m_leftController1;
  private WPI_TalonSRX m_leftController2;
  private WPI_TalonSRX m_rightController1;
  private WPI_TalonSRX m_rightController2;

  private SpeedControllerGroup m_leftControlGroup;
  private SpeedControllerGroup m_rightControlGroup;

  private DifferentialDrive m_robotDrive;

  public Drive(){
    m_leftController1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("leftController1"));
    m_leftController2 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("leftController2"));
    m_rightController1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("rightController1"));
    m_rightController2 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("rightController2"));

    m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);

    m_leftControlGroup.setInverted(false);
    m_rightControlGroup.setInverted(false);

    m_robotDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArcadeDrive());
    
  }

  public void arcadeDrive(double velocity, double heading){
    double dDriveInvert = 1;
    m_robotDrive.arcadeDrive(velocity * dDriveInvert, heading);
  }
}
