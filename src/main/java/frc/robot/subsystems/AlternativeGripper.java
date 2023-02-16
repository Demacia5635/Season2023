// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlternativeGripper extends SubsystemBase {
  private final WPI_TalonSRX gripperOpenner;
  private final DigitalInput closeLimitSwitch;
  private final DigitalInput openLimitSwitch;

  public AlternativeGripper() {
    this.closeLimitSwitch = new DigitalInput(Constants.AlternativeGripper.CLOSE_LIMIT_SWITCH);
    this.openLimitSwitch = new DigitalInput(Constants.AlternativeGripper.OPEN_LIMIT_SWITCH);
    this.gripperOpenner = new WPI_TalonSRX(Constants.AlternativeGripper.GRIPPER_OPENNER_ID);
  }

  /**
   * 
   * @param power power from -1 to 1
   * @return set the power of the motor to the power that he gets * a constants
   */
  public void setPowerToGripper(double power) { // TODO feel free to change the name 
    gripperOpenner.set(power * Constants.AlternativeGripper.MAX_SPEED_GRIPPER_OPENNER);

  }

  /**
   * 
   * @return get the location in pulses
   */

  public double getLocation() {
    return gripperOpenner.getSelectedSensorPosition();
  }

  /**
   * 
   * @return reset the pulses of the motor
   */

  public void resetLocation() {
    gripperOpenner.setSelectedSensorPosition(0);// TODO finds the amount of pulses per meter
  }

  /**
   * 
   * @return true if the gripper is open else it returns false
   */

  public boolean isOpen() {
    return openLimitSwitch.get();
  }

  /**
   * 
   * @return true if the gripper is close else it returns false
   */

  public boolean isClosed() {
    return closeLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("is close", isClosed());
    SmartDashboard.putBoolean("is open", isOpen());

    SmartDashboard.putNumber("target power", 0);
    SmartDashboard.putData("set power",
        new RunCommand(() -> setPowerToGripper(SmartDashboard.getNumber("target power", 0)), this));

    SmartDashboard.putNumber("Location in pulses", getLocation()); // TODO find the number of pulses per meter

    SmartDashboard.putData("reset location", new InstantCommand(() -> resetLocation(), this));

  }
}
