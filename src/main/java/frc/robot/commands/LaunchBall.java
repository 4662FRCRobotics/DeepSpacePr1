/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.commands.PushBall;
import frc.robot.commands.PullBall;
import frc.robot.commands.GripOpen;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class LaunchBall extends CommandGroup {

  private final double I_LOST_THE_GAME = 0.2;

  public LaunchBall() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addSequential(new GripOpen(I_LOST_THE_GAME));
    addSequential(new PushBall(I_LOST_THE_GAME));
    addSequential(new PullBall(I_LOST_THE_GAME));

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    requires(Robot.m_manipulator);
  }
}
