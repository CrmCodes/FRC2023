package frc.robot.commands.TeleOpCmd;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveTeleOpCmd extends CommandBase {

	private final DriveSubsystem driveSubsystem;
	private final Supplier<Double> xValue, yValue, turnValue, isFullSpeed;
	private final Supplier<Boolean> isFieldOriented;
	private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
	/** Creates a new SwerveTeleOpCmd. */
	public SwerveTeleOpCmd(DriveSubsystem driveSubsystem,
			Supplier<Double> xValue, 
			Supplier<Double> yValue, 
			Supplier<Double> turnValue,
			Supplier<Boolean> isFieldOriented,
			Supplier<Double> isFullSpeed) {
		this.driveSubsystem = driveSubsystem;
		this.xValue = xValue;
		this.yValue = yValue;
		this.turnValue = turnValue;
		this.isFieldOriented = isFieldOriented;
		this.isFullSpeed = isFullSpeed;
		this.xLimiter = new SlewRateLimiter(2);
		this.yLimiter = new SlewRateLimiter(2);
		this.turnLimiter = new SlewRateLimiter(2);
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double xSpeed = xValue.get();
		double ySpeed = yValue.get();
		double turnSpeed = turnValue.get();

		xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
		ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
		turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;

		if(isFullSpeed.get() < 0.3){
			xSpeed = xSpeed / 4;
			ySpeed = ySpeed / 4;
		}

		xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		turnSpeed = turnLimiter.calculate(turnSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

		ChassisSpeeds chassisSpeeds;
		if (isFieldOriented.get()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				xSpeed, ySpeed, turnSpeed, driveSubsystem.getRotation2d());
		} else {
			chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
		}

		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		driveSubsystem.setModuleStates(moduleStates);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveSubsystem.stopModules();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
