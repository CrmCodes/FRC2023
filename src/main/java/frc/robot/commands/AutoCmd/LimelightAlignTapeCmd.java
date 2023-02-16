// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.DriveConstants;

public class LimelightAlignTapeCmd extends CommandBase {
  PIDController controller = new PIDController(0.05, 0, 0);
  SlewRateLimiter m_visionRateLimiter = new SlewRateLimiter(0.3);
  DriveSubsystem driveSubsystem;
  LimelightSubsystem limelightSubsystem;
  private double spd;

  public LimelightAlignTapeCmd(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(driveSubsystem, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelightSubsystem.targetDetected()){
      spd = MathUtil.clamp(controller.calculate(limelightSubsystem.MCgetYaw(), 0), -1.0, 1.0);
    } else {
      spd = 0;
    }
    SmartDashboard.putNumber("LLYaw", limelightSubsystem.MCgetYaw());

		double xSpeed = 0;
		double ySpeed = spd;
		double turnSpeed = 0;

		ySpeed = m_visionRateLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 4;

		ChassisSpeeds chassisSpeeds;
		chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		driveSubsystem.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Speed", ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint() || !limelightSubsystem.targetDetected();
  }
}
