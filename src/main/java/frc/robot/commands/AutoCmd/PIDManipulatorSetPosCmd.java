// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class PIDManipulatorSetPosCmd extends CommandBase {
  private final PIDElevatorSubsystem elevatorSubsystem;
  private final PIDArmSubsystem armSubsystem;
  private double elevatorSetpoint, armSetpoint;
  /** Creates a new PIDManipulatorSetPosCmd. */
  public PIDManipulatorSetPosCmd(
    PIDElevatorSubsystem elevatorSubsystem,
    PIDArmSubsystem armSubsystem,
    double elevatorSetpoint,
    double armSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.elevatorSetpoint = elevatorSetpoint;
    this.armSetpoint = armSetpoint;
    addRequirements(elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPos(elevatorSetpoint);
    armSubsystem.setPos(armSetpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
