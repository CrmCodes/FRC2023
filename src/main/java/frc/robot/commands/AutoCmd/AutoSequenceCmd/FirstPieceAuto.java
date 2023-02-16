// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd.AutoSequenceCmd;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCmd.ArmSetPosCmd;
import frc.robot.commands.AutoCmd.ElevatorHasResetCmd;
import frc.robot.commands.AutoCmd.ElevatorSetPosCmd;
import frc.robot.commands.AutoCmd.ExtenderSetPowerCmd;
import frc.robot.commands.AutoCmd.PPSwerveTrajCmd;
import frc.robot.commands.AutoCmd.intakeSetAutoCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FirstPieceAuto extends SequentialCommandGroup {
  /** Creates a new FirstPieceCmd. */
  public FirstPieceAuto(
    DriveSubsystem driveSubsystem,
    PIDElevatorSubsystem elevatorSubsystem,
    PIDArmSubsystem armSubsystem,
    ExtenderSubsystem extenderSubsystem,
    IntakeSubsystem intakeSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PathPlannerTrajectory PPfirstTrajectory = PathPlanner.loadPath("First Piece Path", new PathConstraints(2, 3));
    PPSwerveTrajCmd ppSwerveTrajCmd = new PPSwerveTrajCmd();
    addCommands(
      new intakeSetAutoCmd(intakeSubsystem, -.8),
			// new LimelightAlignTapeCmd(driveSubsystem, limelightSubsystem),
			new ElevatorHasResetCmd(elevatorSubsystem),
			new InstantCommand(() -> driveSubsystem.resetOdometry(PPfirstTrajectory.getInitialHolonomicPose())),
			new ElevatorSetPosCmd(elevatorSubsystem, -0.05),
			new ArmSetPosCmd(armSubsystem, 95),
			new WaitCommand(1.5),
			new ParallelCommandGroup(
				ppSwerveTrajCmd.PPSwerveTrajCommand(driveSubsystem, PPfirstTrajectory),
				new ParallelDeadlineGroup(new WaitCommand(.8), new ExtenderSetPowerCmd(extenderSubsystem, 1.0))),
			new ExtenderSetPowerCmd(extenderSubsystem, 0),
			new ParallelDeadlineGroup(new WaitCommand(.5), new intakeSetAutoCmd(intakeSubsystem, 1.0)),
			new intakeSetAutoCmd(intakeSubsystem, 0),
			new ArmSetPosCmd(armSubsystem, 110),
      new PrintCommand("FIRST PIECE PLACED")
    );
  }

  private PPSwerveTrajCmd PPSwerveTrajCmd() {
    return null;
  }
}
