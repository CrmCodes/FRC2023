// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ElevateCMD extends CommandBase {
  private PIDController pidController = new PIDController(0.00001, 0,
  0.000010);
    private DriveSubsystem driveSubsystem;
    private boolean hasElevated = false;
  /** Creates a new ElevateCMD. */
  public ElevateCMD() {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = driveSubsystem.getPitch();
    double speed = 0;
    // ChassisSpeeds chassisSpeeds;

    if (Math.abs(pitch) > 0 || Math.abs(pitch) < 0) {
        hasElevated = true;

    } else if (pitch == 0){
      hasElevated = false;
    }
    
    if (hasElevated == true){
      speed = MathUtil.clamp(pidController.calculate(pitch, 0), -0.2, 0.2);
    } else {
        speed = 0.25;
    }
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, speed, 0);
    // chassisSpeeds
    driveSubsystem.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

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
