package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

	private final SwerveModule frontLeftModule = new SwerveModule(
		DriveConstants.kFrontLeftDriveMotorId, 
		DriveConstants.kFrontLeftTurnMotorId, 
		DriveConstants.kLeftDriveMotorInverted,
		DriveConstants.kFrontLeftCANCoderId, 
		DriveConstants.kFrontLeftAbsEncoderOffset);
  
	private final SwerveModule frontRightModule = new SwerveModule(
		DriveConstants.kFrontRightDriveMotorId, 
		DriveConstants.kFrontRightTurnMotorId, 
		DriveConstants.kRightDriveMotorInverted, 
		DriveConstants.kFrontRightCANCoderId, 
		DriveConstants.kFrontRightAbsEncoderOffset);

	private final SwerveModule rearLeftModule = new SwerveModule(
		DriveConstants.kRearLeftDriveMotorId, 
		DriveConstants.kRearLeftTurnMotorId, 
		DriveConstants.kLeftDriveMotorInverted, 
		DriveConstants.kRearLeftCANCoderId, 
		DriveConstants.kRearLeftAbsEncoderOffset);

	private final SwerveModule rearRightModule = new SwerveModule(
		DriveConstants.kRearRightDriveMotorId, 
		DriveConstants.kRearRightTurnMotorId, 
		DriveConstants.kRightDriveMotorInverted, 
		DriveConstants.kRearRightCANCoderId, 
		DriveConstants.kRearRightAbsEncoderOffset);

	private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
	private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
			new Rotation2d(0), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), rearLeftModule.getPosition(), rearRightModule.getPosition()});

	private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				zeroHeading();
			} catch (Exception e) {
			}
		}).start();
		SmartDashboard.putData("Field", m_field);
	}

	public void zeroHeading() {
		m_gyro.reset();
	}

	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360);
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}

	public Pose2d getPose2d() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getRotation2d(), 
				new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), rearLeftModule.getPosition(), rearRightModule.getPosition()}, 
				pose);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		odometry.update(getRotation2d(), 
						new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), rearLeftModule.getPosition(), rearRightModule.getPosition()});
		m_field.setRobotPose(odometry.getPoseMeters());
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("pitch", getPitch());
		SmartDashboard.putString("robot location", getPose2d().getTranslation().toString());
	}

	public double getPitch(){
		return m_gyro.getYComplementaryAngle();
	}

	public void stopModules() {
		frontLeftModule.stop();
		frontRightModule.stop();
		rearLeftModule.stop();
		rearRightModule.stop();
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
		frontLeftModule.setDesiredState(desiredStates[0]);
		frontRightModule.setDesiredState(desiredStates[1]);
		rearLeftModule.setDesiredState(desiredStates[2]);
		rearRightModule.setDesiredState(desiredStates[3]);
	}
}
