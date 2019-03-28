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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ArmSetPoint;
import frc.robot.Robot;
import frc.robot.commands.MoveWristJoystick;

import com.ctre.phoenix.ParamEnum;
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

  private final int MODULE_NUMBER = 1;
  private final int BRAKE_FORWARD = 5;
  private final int BRAKE_BACKWARD = 4;

  private int m_iParkEV;
  private int m_iBall1EV;
  private int m_iBall2EV;
  private int m_iBall3EV;
  private int m_iPort1EV;
  private int m_iPort2EV;
  private int m_iPort3EV;
  private int m_iBallCSEV;
  private int m_iPortCSEV;

  private int m_iForwardLimit;
  private int m_iReverseLimit;

  private WPI_TalonSRX m_jointMotor1;
  private WPI_TalonSRX m_jointMotor2;
  private double m_dArmJointPIDP;
  private double m_dArmJointPIDI;
  private double m_dArmJointPIDD;
  private double m_dArmJointPIDTolerance;
  private double m_dArmJointPIDSpeed;
  private double m_dArmJointFeedForward;
  private PIDController m_ArmJointPIDCntl;

  private double m_dTarget;
  private double m_dSpeed;

  private DoubleSolenoid m_jointBrake;
  
  private boolean m_bHasBrake;

  private String m_strMotorString;

  private SpeedControllerGroup m_jointMotorGroup;

  private final double kRAMP_RATE = 0.5;

  public ARMJoint(String motorString, boolean hasBrake){
    this(1, motorString, hasBrake);
  }

  public ARMJoint(int motorCount, String motorString, boolean hasBrake){

    m_iParkEV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.PARK.getStrArmSetPoint() );
    m_iBall1EV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.BALL1.getStrArmSetPoint() );
    m_iBall2EV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.BALL2.getStrArmSetPoint() );
    m_iBall3EV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.BALL3.getStrArmSetPoint() );
    m_iPort1EV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.PORT1.getStrArmSetPoint() );
    m_iPort2EV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.PORT2.getStrArmSetPoint() );
    m_iPort3EV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.PORT3.getStrArmSetPoint() );
    m_iBallCSEV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.BALLCS.getStrArmSetPoint() );
    m_iPortCSEV = Robot.m_robotMap.getARMJoint(motorString, ArmSetPoint.PORTCS.getStrArmSetPoint() );
   
    System.out.format("PARK = %d",m_iParkEV);
    System.out.format("BALL1 = %d",m_iBall1EV);
    System.out.format("BALL2 = %d", m_iBall2EV);
    System.out.format("BALL3 = %d",m_iBall3EV);
    System.out.format("PORT1 = %d",m_iPort1EV);
    System.out.format("PORT2 = %d",m_iPort2EV);
    System.out.format("PORT3 = %d",m_iPort3EV);
    System.out.format("BALLCS = %d",m_iBallCSEV);
    System.out.format("PORTCS = %d",m_iPortCSEV);

    if (motorCount > 2){
      motorCount = 2;
    }
    else if  (motorCount < 1){
      motorCount = 1;
    }

    m_jointMotor1 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber(motorString + MOTORNUMBERONE));
    m_jointMotor1.configFactoryDefault();
    m_jointMotor1.setNeutralMode(NeutralMode.Brake);
    m_jointMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_jointMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    switch (motorString){
      case "wrist":
        m_iForwardLimit = Robot.m_robotMap.getARMJoint(motorString, "fwdLimit");
        m_iReverseLimit = Robot.m_robotMap.getARMJoint(motorString, "revLimit");
        m_jointMotor1.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        m_jointMotor1.configSetParameter(ParamEnum.eFeedbackNotContinuous, 0, 0, 0, 0);
        m_jointMotor1.setSensorPhase(false);
        m_jointMotor1.configForwardSoftLimitThreshold(m_iForwardLimit);
        m_jointMotor1.configReverseSoftLimitThreshold(m_iReverseLimit);
       // m_jointMotor1.configForwardSoftLimitEnable(false);
       // m_jointMotor1.configReverseSoftLimitEnable(false);
       //Not working.
        break;
      
      case "elbow":
        m_jointMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        m_jointMotor1.setSensorPhase(false);
        m_jointMotor1.configClearPositionOnLimitR(true, 0);
        break;
      
      default:
    }
    
    m_jointMotor1.configOpenloopRamp(kRAMP_RATE);
    m_jointMotor1.configClosedloopRamp(kRAMP_RATE);

    /**
     * private double m_dArmJointPIDP;
     * private double m_dArmJointPIDI;
     * private double m_dArmJointPIDD;
     * private double m_dArmJointPIDTolerance;
     * private double m_dArmJointPIDSpeed;
     * 
     * private PIDController m_ArmJointPIDCntl;
     */

    m_dSpeed = 0;
    m_dTarget = 0;
    
    m_dArmJointPIDP = Robot.m_robotMap.getPIDPVal(motorString, 0.2);
    m_dArmJointPIDI = Robot.m_robotMap.getPIDIVal(motorString, 0.0);
    m_dArmJointPIDD = Robot.m_robotMap.getPIDDVal(motorString, 0.0);
    m_dArmJointPIDTolerance = Robot.m_robotMap.getPIDToleranceVal(motorString, 100);
    m_dArmJointFeedForward = Robot.m_robotMap.getPIDFfVal(motorString, 0.1);
    m_dArmJointPIDSpeed = 1;

    m_ArmJointPIDCntl = new PIDController(m_dArmJointPIDP, m_dArmJointPIDI, m_dArmJointPIDD, m_dArmJointFeedForward, new getArmJointEncoder(), new putArmJointSpeed());
    m_ArmJointPIDCntl.setContinuous(false);
    m_ArmJointPIDCntl.setInputRange(Robot.m_robotMap.getMinPIDInput(motorString, -7000), Robot.m_robotMap.getMaxPIDInput(motorString, 10));
    m_ArmJointPIDCntl.setOutputRange(-Math.abs(m_dArmJointPIDSpeed), Math.abs(m_dArmJointPIDSpeed));
    m_ArmJointPIDCntl.setAbsoluteTolerance(m_dArmJointPIDTolerance);

    switch (motorCount){
      case 1:
        m_jointMotorGroup = new SpeedControllerGroup(m_jointMotor1);
        break;
      
      case 2:
        m_jointMotor2 = new WPI_TalonSRX(Robot.m_robotMap.getPortNumber(motorString + MOTORNUMBERTWO));
        m_jointMotorGroup = new SpeedControllerGroup(m_jointMotor1, m_jointMotor2);
        
        break;
      
      default:
    }

    m_jointMotorGroup.setInverted(true);

    m_bHasBrake = hasBrake;
    if(m_bHasBrake){
      m_jointBrake = new DoubleSolenoid(MODULE_NUMBER, BRAKE_FORWARD, BRAKE_BACKWARD);
    }
    m_strMotorString = motorString;
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    switch(m_strMotorString){
      case "wrist":
        setDefaultCommand(new MoveWristJoystick());
        break;
    }
  }

  public void moveJointMotor(double speed) {
   m_jointMotorGroup.set(speed);
   m_dSpeed = speed;
  }

  public void displayJointMotor(){
    SmartDashboard.putNumber(m_strMotorString + "Encoder", m_jointMotor1.getSelectedSensorPosition(0) );
    SmartDashboard.putNumber(m_strMotorString + "Target", m_dTarget);
    SmartDashboard.putNumber(m_strMotorString + "Speed", m_dSpeed);
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

  public void holdPosition() {
    enableArmJointPID(m_jointMotor1.getSelectedSensorPosition(0));
  }

  public void setArmLevel(ArmSetPoint levelName){
    switch(levelName) {
      case PARK:
        enableArmJointPID(m_iParkEV);
        break;
      case BALL1:
        enableArmJointPID(m_iBall1EV);
        break;
      case BALL2:
        enableArmJointPID(m_iBall2EV);
        break;
      case BALL3:
        enableArmJointPID(m_iBall3EV);
        break;
      case PORT1:
        enableArmJointPID(m_iPort1EV);
        break;
      case PORT2:
        enableArmJointPID(m_iPort2EV);
        break;
      case PORT3:
        enableArmJointPID(m_iPort3EV);
        break;
      case BALLCS:
        enableArmJointPID(m_iBallCSEV);
        break;
      case PORTCS:
        enableArmJointPID(m_iPortCSEV);  
      default:
    }
  }

  public void enableArmJointPID(double target){

    m_dTarget = target;
    m_ArmJointPIDCntl.setSetpoint(target);

    if (!m_ArmJointPIDCntl.isEnabled()) {
      m_ArmJointPIDCntl.enable();
    }
  }

  public void disableArmJointPID(){
    m_ArmJointPIDCntl.disable();
  }

  public boolean isArmJointOnTarget(){
    return m_ArmJointPIDCntl.onTarget()
      || (m_dSpeed < 0 && m_jointMotor1.getSensorCollection().isFwdLimitSwitchClosed())
      || (m_dSpeed > 0 && m_jointMotor1.getSensorCollection().isRevLimitSwitchClosed());
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