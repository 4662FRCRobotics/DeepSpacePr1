/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class RGBLights extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static I2C rgbController;

  public RGBLights() {
    rgbController = new I2C(I2C.Port.kMXP, 8);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void UpdateLEDs(String WriteString) {

    WriteString += UpdateColor();

    char[] CharArray = WriteString.toCharArray();
    byte[] RobotStatus = new byte [CharArray.length];

    for (int i = 0; i < CharArray.length; i++){
      RobotStatus[i] = (byte) CharArray[i];
    }

    rgbController.writeBulk(RobotStatus, RobotStatus.length);
  }
  public String UpdateColor() {
    Alliance alliance = DriverStation.getInstance().getAlliance();
    String allianceColor = ",INVALID";
    switch (alliance){
      case Blue:
        allianceColor = ",BLUE";
        break;
      case Red:
        allianceColor = ",RED";
        break;
      default:
        allianceColor = ",INVALID";
        break;
    }
    return allianceColor;
  }
}
