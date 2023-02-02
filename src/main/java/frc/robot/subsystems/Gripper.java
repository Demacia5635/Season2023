// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private TalonSRX motor;
  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;
  /** Creates a new Gripper. */
  public Gripper(TalonSRX motor, DigitalInput limitSwitch1, DigitalInput limitSwitch2) {
    this.motor = motor;
    this.limitSwitch1 = limitSwitch1;
    this.limitSwitch2 = limitSwitch2;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open(){
    motor.set(ControlMode.PercentOutput, Constants.GripperConstants.OPEN_POWER);
    if(limitSwitch1.get()){
      motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void close(){
    motor.set(ControlMode.PercentOutput, Constants.GripperConstants.CLOSE_POWER);
    if(limitSwitch2.get()){
      motor.set(ControlMode.PercentOutput, 0);
    }
  }
}
