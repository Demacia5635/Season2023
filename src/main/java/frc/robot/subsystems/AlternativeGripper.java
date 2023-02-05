// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlternativeGripper extends SubsystemBase {
  private final WPI_TalonSRX gripperOpenner;
  public AlternativeGripper() {

    this.gripperOpenner = new WPI_TalonSRX(Constants.AlternativeGripper.GRIPPER_OPENNER_ID);
  }


  public void setPowerToGripper(double power){
    gripperOpenner.set(power);

  }

  public double getLocation(){
    return gripperOpenner.getSelectedSensorPosition();
  }

  public void resetLocation(){
      gripperOpenner.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("set power", new RunCommand(()->
        setPowerToGripper(SmartDashboard.getNumber("target power", 0)), this));
        
    SmartDashboard.putNumber("Location", getLocation()); // TODO find the number of pulses from one end to other end

    SmartDashboard.putData("reset location", new RunCommand(()->
    resetLocation(), this)); // TODO make it a button


  }
}
