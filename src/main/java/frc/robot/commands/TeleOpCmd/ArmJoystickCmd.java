// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOpCmd;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDArmSubsystem;

public class ArmJoystickCmd extends CommandBase {
  private final PIDArmSubsystem armSubsystem;
  private XboxController joystick;
  /** Creates a new ArmJoystickCmd. */
  public ArmJoystickCmd(PIDArmSubsystem armSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Offset = Math.abs(joystick.getLeftY()) > 0.1?  -joystick.getLeftY() : 0;
    Offset = Offset * 40;

    if(joystick.getAButton()){
      armSubsystem.setPos(90 + Offset);
    } else if(joystick.getBButton()){
      armSubsystem.setPos(40 + Offset);
    } else if(joystick.getXButton()){
      armSubsystem.setPos(-90 - Offset);
    } else if(joystick.getYButton()){
      armSubsystem.setPos(0 + Offset);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
