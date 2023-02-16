// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDArmSubsystem;

public class ArmSetPosCmd extends CommandBase {
  private final PIDArmSubsystem armSubsystem;
  private double armSetpoint;
  /** Creates a new PIDManipulatorSetPosCmd. */
  public ArmSetPosCmd(
    PIDArmSubsystem armSubsystem,
    double armSetpoint) {
    this.armSubsystem = armSubsystem;
    this.armSetpoint = armSetpoint;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
