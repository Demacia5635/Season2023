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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private TalonSRX motor;
  /** Creates a new Gripper. */
  public Gripper(int motorId) {
    motor = new TalonSRX(motorId);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, motorId);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, motorId);
  }

  private void setPower(double power){
    motor.set(TalonSRXControlMode.PercentOutput, power);
  }

  private boolean getLimSwitchClose(){
    return(motor.isRevLimitSwitchClosed() == 1);
  }

  private boolean getLimSwitchOpen(){
    return(motor.isFwdLimitSwitchClosed() == 1);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open(){
    motor.set(ControlMode.PercentOutput, GripperConstants.OPEN_POWER);
  }

  public void close(){
    motor.set(ControlMode.PercentOutput, GripperConstants.CLOSE_POWER);
  }
  
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("limSwitch close", this::getLimSwitchClose, null);
    builder.addBooleanProperty("limSwitch open", this::getLimSwitchOpen, null);
  }

  public Command getOpenCommand(){
    return new StartEndCommand(this::open, ()-> setPower(0) , this).until(this::getLimSwitchOpen);
  }

  public Command getCloseCommand(){
    return new StartEndCommand(this::close, ()-> setPower(0) , this).until(this::getLimSwitchClose);
  }

}
