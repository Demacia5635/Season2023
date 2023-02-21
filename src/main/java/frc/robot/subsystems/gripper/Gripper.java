// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private TalonSRX motor;
  /** Creates a new Gripper. */
  public Gripper(int motorId) {
    motor = new TalonSRX(motorId);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, motorId);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, motorId);
  }

  /**
   * Sets gripper motor power.
   * @param power
   */
  private void setPower(double power){
    motor.set(TalonSRXControlMode.PercentOutput, power);
  }

  /**
   * Returns the close motion activated limit switch`s state. true = pressed.
   * @return Lim switch state
   */
  private boolean isLimitSwitchClose(){
    return motor.isRevLimitSwitchClosed() == 1;
  }

  /**
   * Returns the open motion activated limit switch`s state. true = pressed.
   * @return Lim switch state
   */
  private boolean isLimitSwitchOpen(){
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  /**
   * Opens the gripper when called.
   */
  private void open(){
    motor.set(ControlMode.PercentOutput, GripperConstants.OPEN_POWER);
  }

  /** 
   * Closes the gripper when called.
   */
  private void close(){
    motor.set(ControlMode.PercentOutput, GripperConstants.CLOSE_POWER);
  }
  

  /**
   * Creates a new StartEndCommand to open the gripper with the end condition of the limit switch being pressed
   * @return Open COMMAND
   */
  public Command getOpenCommand(){
    return new StartEndCommand(this::open, ()-> setPower(0), this).until(this::isLimitSwitchOpen);
  }

  /**
   * Creates a new StartEndCommand to Close the gripper with the end condition of the limit switch being pressed
   * @return Close COMMAND
   */
  public Command getCloseCommand(){
    return new StartEndCommand(this::close, ()-> {} , this).until(this::isLimitSwitchClose);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.addBooleanProperty("Limit Switch close", this::isLimitSwitchClose, null);
    builder.addBooleanProperty("Limit Switch open", this::isLimitSwitchOpen, null);
    SmartDashboard.putData("Open Gripper",  new StartEndCommand(this::open, ()-> setPower(0) , this).until(this::isLimitSwitchOpen));
    SmartDashboard.putData("Close Gripper",  new StartEndCommand(this::close, ()-> setPower(0) , this).until(this::isLimitSwitchClose));
  }
}
