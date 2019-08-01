/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// I just lost the game

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 // public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static RobotMap m_robotMap;
  public static Drive m_drive;
  public static Vision m_vision;
  public static ARMJoint m_elbowJoint;
  public static ARMJoint m_wristJoint;
  public static Manipulator m_manipulator;
  public static ManipulatorIntake m_manipulatorIntake;
  public static RearClimb m_rearClimb;
  public static Autonomous m_autonomous;
  public static RGBLights m_rgbLights;
  public static OI m_oi;

  public static GetNextCmd m_autonomousCommand;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_robotMap = new RobotMap("/home/lvuser/robot.xml");
    m_drive = new Drive();
    m_vision = new Vision();
    m_elbowJoint = new ARMJoint("elbow", true);
    m_wristJoint = new ARMJoint("wrist", false);
    m_manipulator = new Manipulator();
    m_manipulatorIntake = new ManipulatorIntake();
    m_rearClimb = new RearClimb();
    m_autonomous = new Autonomous();
    m_rgbLights = new RGBLights();
    m_oi = new OI();

    /** m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);*/
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_elbowJoint.displayJointMotor();
    m_wristJoint.displayJointMotor();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_vision.turnVisionOff();
    m_rgbLights.UpdateLEDs("DISABLED");
    //m_rgbLights.UpdateColor();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
     // m_autonomousCommand.start();
    //}

    m_rgbLights.UpdateLEDs("AUTO");

    m_autonomous.searchAutoXml();
    m_autonomousCommand = new GetNextCmd();
    m_autonomousCommand.start();
    //m_rgbLights.UpdateColor();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //if (m_autonomousCommand != null) {
     // m_autonomousCommand.cancel();
    //}

    m_rgbLights.UpdateLEDs("TELEOP");
    //m_rgbLights.UpdateColor();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
