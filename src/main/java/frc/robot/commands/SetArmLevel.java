/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.ArmSetPoint;
import frc.robot.Robot;

public class SetArmLevel extends CommandGroup {

  public SetArmLevel(ArmSetPoint armLevel) {
    requires(Robot.m_wristJoint);
    requires(Robot.m_elbowJoint);

    addParallel(new SetElbowLevel(armLevel));
    //addSequential(new SetWristLevel(armLevel));
  }
}
