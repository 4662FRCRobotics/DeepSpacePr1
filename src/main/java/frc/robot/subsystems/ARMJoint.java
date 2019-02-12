/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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

  private final String PARK = "park";
  private final String BALL1 = "ball1";
  private final String BALL2 = "ball2";
  private final String BALL3 = "ball3";
  private final String PORT1 = "port1";
  private final String PORT2 = "port2";
  private final String PORT3 = "port3";
  private final int MODULE_NUMBER = 1;
  private final int BRAKE_FORWARD = 0;
  private final int BRAKE_BACKWARD = 1;

  private int m_iParkEV;
  private int m_iBall1EV;
  private int m_iBall2EV;
  private int m_iBall3EV;
  private int m_iPort1EV;
  private int m_iPort2EV;
  private int m_iPort3EV;

  private WPI_TalonSRX m_jointMotor1;
  private WPI_TalonSRX m_jointMotor2;
  private double m_dArmJointPIDP;
  private double m_dArmJointPIDI;
  private double m_dArmJointPIDD;
  private double m_dArmJointPIDTolerance;
  private double m_dArmJointPIDSpeed;
  private double m_dArmJointFeedForward;
  private PIDController m_ArmJointPIDCntl;

  private  DoubleSolenoid m_jointBrake;
  
  private boolean m_bHasBrake;

  private SpeedControllerGroup m_jointMotorGroup;

  public ARMJoint(String motorString, boolean hasBrake){
    this(1, motorString, hasBrake);
  }

  public ARMJoint(int motorCount, String motorString, boolean hasBrake){

    m_iParkEV = Robot.m_robotMap.getARMJoint(motorString, PARK);
    m_iBall1EV = Robot.m_robotMap.getARMJoint(motorString, BALL1 );
    m_iBall2EV = Robot.m_robotMap.getARMJoint(motorString, BALL2 );
    m_iBall3EV = Robot.m_robotMap.getARMJoint(motorString, BALL3 );
    m_iPort1EV = Robot.m_robotMap.getARMJoint(motorString, PORT1 );
    m_iPort2EV = Robot.m_robotMap.getARMJoint(motorString, PORT2 );
    m_iPort3EV = Robot.m_robotMap.getARMJoint(motorString, PORT3 );
   
    System.out.format("PARK = %d",m_iParkEV);
    System.out.format("BALL1 = %d",m_iBall1EV);
    System.out.format("BALL2 = %d", m_iBall2EV);
    System.out.format("BALL3 = %d",m_iBall3EV);
    System.out.format("PORT1 = %d",m_iPort1EV);
    System.out.format("PORT2 = %d",m_iPort2EV);
    System.out.format("PORT3 = %d",m_iPort3EV);

    if (motorCount > 2){
      motorCount = 2;
    }
    else if  (motorCount < 1){
      motorCount = 1;
    }

    m_jointMotor1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber(motorString + MOTORNUMBERONE));
    m_jointMotor1.setNeutralMode(NeutralMode.Brake);
    m_jointMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_jointMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_jointMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_jointMotor1.setSensorPhase(false);

    /**
     * private double m_dArmJointPIDP;
     * private double m_dArmJointPIDI;
     * private double m_dArmJointPIDD;
     * private double m_dArmJointPIDTolerance;
     * private double m_dArmJointPIDSpeed;
     * private double m_dArmJointFeedForward;
     * private PIDController m_ArmJointPIDCntl;
     */

    m_dArmJointPIDP = Robot.m_robotMap.getPIDPVal(motorString, 0.2);
    m_dArmJointPIDI = Robot.m_robotMap.getPIDIVal(motorString, 0.0);
    m_dArmJointPIDD = Robot.m_robotMap.getPIDDVal(motorString, 0.0);
    m_dArmJointPIDTolerance = Robot.m_robotMap.getPIDToleranceVal(motorString, 100);
    m_dArmJointPIDSpeed = 1;

    m_ArmJointPIDCntl = new PIDController(m_dArmJointPIDP, m_dArmJointPIDI, m_dArmJointPIDD, new getArmJointEncoder(), new putArmJointSpeed());
    m_ArmJointPIDCntl.setContinuous(false);

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

    m_bHasBrake = hasBrake;
    if(m_bHasBrake){
      m_jointBrake = new DoubleSolenoid(MODULE_NUMBER, BRAKE_FORWARD, BRAKE_BACKWARD);
    }

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void moveJointMotor(double speed) {
   m_jointMotorGroup.set(speed); 
  }

  public void setBrakeForward(){
    if(m_bHasBrake){
      m_jointBrake.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void setBrakeBackward(){
    if(m_bHasBrake){
      m_jointBrake.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void setBrakeOff(){
    if(m_bHasBrake){
      m_jointBrake.set(DoubleSolenoid.Value.kOff);
    }
  }

  public void enableArmJointPID(double target){

    m_ArmJointPIDCntl.setOutputRange(-Math.abs(m_dArmJointPIDSpeed), Math.abs(m_dArmJointPIDSpeed));
    m_ArmJointPIDCntl.setSetpoint(target);
    m_ArmJointPIDCntl.setAbsoluteTolerance(m_dArmJointPIDTolerance);
    m_ArmJointPIDCntl.enable();

  }

  public void disableArmJointPID(){
    m_ArmJointPIDCntl.disable();
  }

  public boolean isArmJointOnTarget(){
    return m_ArmJointPIDCntl.onTarget();
  }

  private class getArmJointEncoder implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
      
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return m_jointMotor1.getSelectedSensorPosition(0);
	  }

  }

  private class putArmJointSpeed implements PIDOutput{

    @Override
    public void pidWrite(double output) {
      moveJointMotor(output);
    }

  }
}