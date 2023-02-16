// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleOpCmd;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorDetectCVSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class ColorTracking extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ColorDetectCVSubsystem colorDetectSubsystem;

  // public double mc_Aim = 0;
  // public double mc_Distance = 0;
  // public double dc_Aim = 0.5;
  // public double dc_Distance = 0.6;

  /** Creates a new ColorTracking. */
  public ColorTracking(DriveSubsystem driveSubsystem, ColorDetectCVSubsystem colorDetectSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.colorDetectSubsystem = colorDetectSubsystem;

    addRequirements(driveSubsystem, colorDetectSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // PIDController kcDcontroller = new PIDController(-4. , 0, 0);
    // PIDController kcAimcontroller = new PIDController(4, 0, 0);

    // mc_Distance = kcDcontroller.calculate(colorDetectSubsystem.centerY, dc_Distance);
    // mc_Aim = kcAimcontroller.calculate(colorDetectSubsystem.centerX, dc_Aim);
    
    // SmartDashboard.putNumber("Color Aim : ", mc_Aim);
    // SmartDashboard.putNumber("Color Distance : ", mc_Distance);

    // if (colorDetectSubsystem.centerY < 0.4 && colorDetectSubsystem.centerY > 0.38){
    //   driveSubsystem.setMotor(-colorDetectSubsystem.mc_Distance * 0,0);
    // } else {
    //   driveSubsystem.setMotor(colorDetectSubsystem.mc_Distance,0);
    // }
    // driveSubsystem.setMotor(-colorDetectSubsystem.mc_Aim,-colorDetectSubsystem.mc_Distance);
    
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
