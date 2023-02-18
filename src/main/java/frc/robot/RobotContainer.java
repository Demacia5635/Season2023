// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.chassis.GoUpRamp;
import frc.robot.commands.chassis.GotoCommunity;
import frc.robot.commands.chassis.GotoLoadingZone;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.gripper.Grip;
import frc.robot.commands.parallelogram.GoToAngle;
import frc.robot.commands.parallelogram.GoToBack;
import frc.robot.commands.parallelogram.PickUp;
import frc.robot.commands.parallelogram.PutGamepiece;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final Chassis chassis;
    private static RobotContainer instance;
    public Parallelogram parallelogram;
    private final Gripper gripper;

    public final class RobotFinalCommands {
        
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        chassis = new Chassis();
        parallelogram = new Parallelogram();
        chassis.setDefaultCommand(new Drive(chassis, controller.getHID()));
        SmartDashboard.putData((Sendable) chassis.getDefaultCommand());
        gripper = new Gripper(GripperConstants.MOTOR_ID);
        SmartDashboard.putData(gripper);
        configureButtonBindings();
    }

    /**
     * Returns the instance of the RobotContainer class.
     * 
     * @return the instance of the RobotContainer class.
     */
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
     * or {@link XboxController}), and then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        Command load = new GotoLoadingZone(chassis, controller.getHID())
        .alongWith(new PickUp(parallelogram, chassis))
        .andThen(new Grip(gripper), new GoToBack(parallelogram));

        Command unload = new GotoNodes(chassis, controller.getHID())
        .alongWith(new PutGamepiece(parallelogram, chassis))
        .andThen(new Grip(gripper), new GoToBack(parallelogram));

        Command loadIfInPlace = new PickUp(parallelogram, chassis)
        .andThen(new Grip(gripper), new GoToBack(parallelogram));

        Command unloadIfInPlace = new PutGamepiece(parallelogram, chassis)
        .andThen(new Grip(gripper), new GoToBack(parallelogram));

        controller.a().onTrue(load);
        controller.b().onTrue(unload);
        controller.y().onTrue(new GotoCommunity(chassis, controller.getHID()));
        controller.x().onTrue(new GoUpRamp(chassis, 1.5));

        controller.leftBumper().onTrue(loadIfInPlace);
        controller.rightBumper().onTrue(unloadIfInPlace);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
