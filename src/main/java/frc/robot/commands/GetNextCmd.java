/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class GetNextCmd extends CommandGroup {
  /**
   * Add your docs here.
   */

  public GetNextCmd() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

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

    String command = "";

    do {
      command = Robot.m_autonomous.getNextCmd();
      ProcessCommand(command);
    } while (!Robot.m_autonomous.isFinished());

    if (command != ""){
      
      addSequential(new StartGetNextCommand()); // :)
    }else{
      System.out.println("exiting command loop");
    }
  }

  private void ProcessCommand(String command){
    System.out.println("Scheduled Command: " + command);
    switch (command) {
      case "wait":
        double timeout = Robot.m_autonomous.getDoubleCommandValue();
        System.out.println("Wait value: " + timeout);
        addSequential (new WaitCommand(timeout));
        break;
      
      case "timedMove":
        double duration = Robot.m_autonomous.getDoubleCommandValue();
        System.out.println("Timed moved value: " + duration);
        addSequential (new TimedMove(duration));
        break;

      case "turnAngle":
        double angle = Robot.m_autonomous.getDoubleCommandValue();
        System.out.println("Turn angle value: " + angle);
        addSequential (new TurnAngle(angle));
        break;

      default:
        System.out.println("Unrecognized command: " + command);
    }
  }
}
