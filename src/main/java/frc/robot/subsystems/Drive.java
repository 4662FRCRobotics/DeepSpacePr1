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

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;


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

  private final double kRAMP_RATE = 1.0;
  private final double kENCODER_PULSES_PER_REV = 1;
  private final double kGEARBOX_REDUCTION = (50/12) * (60/14);
  private final double kTIRE_SIZE = 7.9;

  private AHRS m_gyroAndCollison;
  private PIDController m_turnAngle;
  private double m_dTurnAngleP;
  private double m_dTurnAngleI;
  private double m_dTurnAngleD;
  private double m_dTurnAngleTolerance;
  private double m_dAngle;

  private PIDController m_keepHeading;
	private double m_dkeepHeadingP;
	private double m_dkeepHeadingI;
	private double m_dkeepHeadingD;
	private double m_dkeepHeadingTolerance;
  private volatile double m_dSteeringHeading;

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

    m_leftController1.setSmartCurrentLimit(40);
    m_leftController2.setSmartCurrentLimit(40);
    m_rightController1.setSmartCurrentLimit(40);
    m_rightController2.setSmartCurrentLimit(40);

    m_leftControlGroup = new SpeedControllerGroup(m_leftController1, m_leftController2);
    m_rightControlGroup = new SpeedControllerGroup(m_rightController1, m_rightController2);

    m_leftControlGroup.setInverted(false);
    m_rightControlGroup.setInverted(false);

    m_robotDrive = new DifferentialDrive(m_leftControlGroup, m_rightControlGroup);

    m_leftEncoder1 = m_leftController1.getEncoder();
    m_rightEncoder1 = m_rightController1.getEncoder();

    m_gyroAndCollison = new AHRS(SPI.Port.kMXP);
    m_dTurnAngleP = Robot.m_robotMap.getPIDDVal("TurnAngle", 0.2);
    m_dTurnAngleI = Robot.m_robotMap.getPIDDVal("TurnAngle", 0.0);
    m_dTurnAngleD = Robot.m_robotMap.getPIDDVal("TurnAngle", 0.4);
    m_turnAngle = new PIDController(m_dTurnAngleP, m_dTurnAngleI, m_dTurnAngleD, new getSourceAngle(), new putOutputTurn());
    m_dTurnAngleTolerance = Robot.m_robotMap.getPIDDVal("TurnAngle", 2);
    m_dAngle = 0;

    m_dkeepHeadingP = Robot.m_robotMap.getPIDPVal("keepHeading", 0.2);
		m_dkeepHeadingI = Robot.m_robotMap.getPIDIVal("keepHeading", 0.0);
		m_dkeepHeadingD = Robot.m_robotMap.getPIDDVal("keepHeading", 0.4);
		m_keepHeading = new PIDController(m_dkeepHeadingP, m_dkeepHeadingI, m_dkeepHeadingD, new getSourceCamera(), new putCameraHeading() );
		m_dkeepHeadingTolerance = Robot.m_robotMap.getPIDToleranceVal("keepHeading", 2);
		m_dSteeringHeading = 0;

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
    SmartDashboard.putNumber("Gyro", m_gyroAndCollison.getAngle());
  
  }

  private double getGyroAngle(){
    return m_gyroAndCollison.getAngle();
  }

  //*******************************************************************************************
  //this block is for angle pid control.
  //********************************************************************************************

  public void disableTurnAngle(){
    m_turnAngle.disable();
  }

  public void setTurnAngle(double angle){
    setTurnAngle(angle, 0.75);
  }

  public void setTurnAngle(double angle, double throttle){
    m_turnAngle.reset();
    m_gyroAndCollison.zeroYaw();
    m_turnAngle.setInputRange(-180f, 180f);
    m_turnAngle.setOutputRange(-throttle, throttle);
    m_turnAngle.setPID(m_dTurnAngleP, m_dTurnAngleI, m_dTurnAngleD);
    m_turnAngle.setAbsoluteTolerance(m_dTurnAngleTolerance);
    m_turnAngle.setContinuous(false);
    m_turnAngle.setSetpoint(angle);
    m_turnAngle.enable();

  }

  public boolean turnAngleOnTarget(){
    return m_turnAngle.onTarget();
  }

  public void driveKeepHeading(double throttle) {
    arcadeDrive(throttle, m_dSteeringHeading);
  }
  
  //disable the keep heading pid
  public void disableKeepHeading() {
    m_keepHeading.disable();
  }

  public void setKeepHeading() {
    m_keepHeading.reset();
    m_keepHeading.setInputRange(-100.0f, 100.0f);
    m_keepHeading.setOutputRange(-.75, .75);
    m_keepHeading.setPID(m_dkeepHeadingP, m_dkeepHeadingI, m_dkeepHeadingD);
    m_keepHeading.setAbsoluteTolerance(m_dkeepHeadingTolerance);
    m_keepHeading.setContinuous(true);
    m_keepHeading.setSetpoint(0.0);
    m_keepHeading.enable();
  }

  

  private class getSourceAngle implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return getGyroAngle();
    }
	}

  private class getSourceCamera implements PIDSource {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      return Robot.m_vision.getTargetCenter();
    }
	}
  
  private class putOutputTurn implements PIDOutput{

	  @Override
	  public void pidWrite(double output) {
      arcadeDrive(0, output);
	  }

  }

  private class putCameraHeading implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			m_dSteeringHeading = output;
		}
    	
    }
}