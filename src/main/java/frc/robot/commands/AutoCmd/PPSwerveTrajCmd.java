// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCmd;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PPSwerveTrajCmd {
    public PPSwerveControllerCommand PPSwerveTrajCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory trajectory){
		PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
		PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
		PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
		PPSwerveControllerCommand PPswerveControllerCommand = new PPSwerveControllerCommand(
			trajectory, 
			driveSubsystem::getPose2d,
			DriveConstants.kDriveKinematics,
			xController, 
			yController, 
			thetaController, 
			driveSubsystem::setModuleStates,
			false, 
			driveSubsystem);

		return PPswerveControllerCommand;

	}
}
