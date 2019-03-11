/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class ManipulatorIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX m_intakeMotor;


  public ManipulatorIntake() {
    m_intakeMotor = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber("intake"));
  }

  @Override
  public void initDefaultCommand() {
  }

  public void setMotorSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
}
