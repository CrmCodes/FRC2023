package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
	private final CANSparkMax driveMotor;
	private final CANSparkMax turnMotor;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;

	private final PIDController turnPidController;

	private final CANCoder absoluteEncoder;

	public SwerveModule(
		int driveMotorId, 
		int turnMotorId, 
		boolean driveMotorInverted,
		int absoluteEncoderId,
		double absoluteEncoderOffset
	) {

		absoluteEncoder = new CANCoder(absoluteEncoderId);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);

		driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
		turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		turnEncoder = turnMotor.getEncoder();

		driveMotor.restoreFactoryDefaults();
		turnMotor.restoreFactoryDefaults();

		driveMotor.setInverted(driveMotorInverted);
		turnMotor.setInverted(DriveConstants.kTurnMotorInverted);

		driveMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setIdleMode(IdleMode.kBrake);

		driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
		driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);
		turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnPositionConversionFactor);
		turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnVelocityConversionFactor);

		turnPidController = new PIDController(ModuleConstants.kPTurn, 0, 0);
		turnPidController.enableContinuousInput(-Math.PI, Math.PI);

		resetEncoders();
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition();
	}

	public double getTurnPosition() {
		// return turnEncoder.getPosition();
		return getAbsoluteEncoderRad();
	}

	public double getDriveVelocity() {
		return driveEncoder.getVelocity();
	}

	// public double getTurnVelocity() {
	// 	return turnEncoder.getVelocity();
		// return absoluteEncoder.getVelocity();
	// }

	public double getAbsoluteEncoderRad() {
		return -1 * Math.toRadians(absoluteEncoder.getAbsolutePosition());
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		turnEncoder.setPosition(getAbsoluteEncoderRad());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			getDrivePosition(), new Rotation2d(getTurnPosition()));
	  }

	public void setDesiredState(SwerveModuleState state) {
		if (Math.abs(state.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}
		state = SwerveModuleState.optimize(state, getState().angle);
		driveMotor.set(state.speedMetersPerSecond / 4.85);
		turnMotor.set(turnPidController.calculate(getTurnPosition(), state.angle.getRadians()));
		SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
		// SmartDashboard.putNumber("turnID" + turnMotor.getDeviceId(), turnEncoder.getPosition());
	}

	public void stop() {
		driveMotor.set(0);
		turnMotor.set(0);
	}
}
