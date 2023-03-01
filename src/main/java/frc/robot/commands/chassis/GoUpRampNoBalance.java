// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.UtilsGeneral;

public class GoUpRampNoBalance extends CommandBase {

  private final Chassis chassis;
    private final double startVelocity;
    private double velocity;
    private int count;
    private double angle;

    private static final double DEADBAND = 4;
    private static final int COUNT_MINIMUM = 50; // the minimum count to be on the ramp to stop the robot

    /**
     * Creates a new GoUpRamp command.
     * 
     * @param chassis  The chassis subsystem
     * @param velocity The velocity to go up the ramp
     */
    public GoUpRampNoBalance(Chassis chassis, double velocity) {
        this.chassis = chassis;
        this.startVelocity = velocity;

        addRequirements(chassis);
    }

    /**
     * Deadbands the angle to prevent the robot from going up and down the ramp.
     * 
     * @param angle The angle to deadband
     * @return The deadbanded angle
     */
    private static double deadbandAngle(double angle) {
        if (Math.abs(angle) < DEADBAND) {
            return 0;
        }
        return angle;
    }

   

    @Override
    public void initialize() {
        count = 0;
        velocity = startVelocity;
    }

    @Override
    public void execute() {
        angle = chassis.getUpRotation();
        chassis.setAngleAndVelocity(velocity, 0, Math.toRadians(Math.toRadians(UtilsGeneral.isRedAlliance() ? 45 : 235)));
        if(deadbandAngle(angle) != 0){
            count++;                                                                                      
        }

        System.out.println(count);
    }
//      //
    @Override
    public boolean isFinished() {
        System.out.println("Vel: " + velocity + " cur vel: " + chassis.getVelocity() + " time: " + count);
        return count > 35;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
