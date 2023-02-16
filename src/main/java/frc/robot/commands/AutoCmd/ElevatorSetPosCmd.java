// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class ElevatorSetPosCmd extends CommandBase {
  private final PIDElevatorSubsystem elevatorSubsystem;
  private double elevatorSetpoint;
  /** Creates a new PIDManipulatorSetPosCmd. */
  public ElevatorSetPosCmd(
    PIDElevatorSubsystem elevatorSubsystem,
    double elevatorSetpoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorSetpoint = elevatorSetpoint;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPos(elevatorSetpoint);

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
