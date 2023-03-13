// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.chassis.LeaveCommunity;
import frc.robot.commands.chassis.RampTest;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.parallelogram.Parallelogram;

/** Add your docs here. */
public class GenerateAutonomous {
    private SendableChooser<Boolean> placeGamePeace;
    private SendableChooser<Boolean> exitCommunity;
    private SendableChooser<Boolean> climb;
    private GotoNodes gotoNodes;
    private LeaveCommunity leaveCommunity;
    private Gripper gripper;
    private Chassis chassis;
    Parallelogram parallelogram;

    /** Creates a new GenerateAutonomous. */
    public GenerateAutonomous(GotoNodes gotoNodes, LeaveCommunity leaveCommunity, Gripper gripper,
            Parallelogram parallelogram, Chassis chassis) {
        this.gotoNodes = gotoNodes;
        this.leaveCommunity = leaveCommunity;
        this.gripper = gripper;
        this.chassis = chassis;
        this.parallelogram = parallelogram;

        placeGamePeace = new SendableChooser<>();
        placeGamePeace.setDefaultOption("Place", true);
        placeGamePeace.addOption("DontPlace", false);
        SmartDashboard.putData("PlacegamePeace Auto", placeGamePeace);

        exitCommunity = new SendableChooser<>();
        exitCommunity.setDefaultOption("Leave", true);
        exitCommunity.addOption("Dont Leave", false);
        SmartDashboard.putData("LeaveCommunity Auto", exitCommunity);

        climb = new SendableChooser<>();
        climb.setDefaultOption("Climb", true);
        climb.addOption("Dont Climb", false);
        SmartDashboard.putData("Climb Auto", climb);
    }

    public SequentialCommandGroup getAutonomous() {
        boolean isClimb = climb.getSelected().equals(true);
        boolean isPlace = placeGamePeace.getSelected().equals(true);
        boolean isLeave = exitCommunity.getSelected().equals(true);
        SequentialCommandGroup autonomous = new SequentialCommandGroup(
                isPlace ? gotoNodes.andThen(gripper.getOpenCommand())
                        .withTimeout(isClimb ? Constants.AUTO_NODES_MAX_TIME : 15)
                        : new InstantCommand(() -> System.out.println("didnt place Game Peace")),
                new InstantCommand(() -> System.out.println("Auto test EEEE")),
                (isLeave ? leaveCommunity : new InstantCommand(() -> System.out.println("didnt leave")))
                        .alongWith(parallelogram.getGoBackCommand()),
                isClimb ? (isLeave ? new RampTest(chassis)
                        : new InstantCommand(() -> System.out.println("wanted to climb but leave was false")))
                        : new InstantCommand(() -> System.out.println("didnt climb")));
        autonomous.setName("Auto");
        return autonomous;
    }
}
