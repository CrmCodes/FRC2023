package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
		public static final double kDriveMotorGearRatio = 1 / 7.0;
		public static final double kTurnMotorGearRatio = 1 / 27.5;
		public static final double kDrivePositionConversionFactor = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
		public static final double kTurnPositionConversionFactor = kTurnMotorGearRatio * 2 * Math.PI;
		public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60;
		public static final double kTurnVelocityConversionFactor = kTurnPositionConversionFactor / 60;
		public static final double kPTurn = 0.5;
	}

	public static final class DriveConstants {

		public static final double kTrackWidth = 0.59;
		public static final double kWheelBase = 0.59;
		public static final Translation2d kFrontLeftPosition = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
		public static final Translation2d kFrontRightPosition = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
		public static final Translation2d kRearLeftPosition = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
		public static final Translation2d kRearRightPosition = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				kFrontLeftPosition,
				kFrontRightPosition,
				kRearLeftPosition,
				kRearRightPosition
		);

		public static final int kFrontLeftDriveMotorId = 1;
		public static final int kFrontRightDriveMotorId = 4;
		public static final int kRearRightDriveMotorId = 5;
		public static final int kRearLeftDriveMotorId = 7;

		public static final int kFrontLeftTurnMotorId = 2;
		public static final int kFrontRightTurnMotorId = 3;
		public static final int kRearRightTurnMotorId = 6;
		public static final int kRearLeftTurnMotorId = 8;

		public static final boolean kTurnMotorInverted = true;

		public static final boolean kLeftDriveMotorInverted = true;
		public static final boolean kRightDriveMotorInverted = false;

		public static final int kFrontLeftCANCoderId = 9;
		public static final int kFrontRightCANCoderId = 10;
		public static final int kRearRightCANCoderId = 11;
		public static final int kRearLeftCANCoderId = 12;

		public static final double kFrontLeftAbsEncoderOffset = -322;
		public static final double kFrontRightAbsEncoderOffset = -84;
		public static final double kRearLeftAbsEncoderOffset = -352;
		public static final double kRearRightAbsEncoderOffset = -149;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;
		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
				kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
	}

	public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

	public static final class ElevatorConstants {
		public static final double kMotorGearRatio = 1.0/10.0;
		public static final double kSprocketRatio = 22.0/18.0;
		public static final double kSprocketRadius = 0.018;
		
		public static final double kTopSetPoint = 0.8;
		public static final double kBottomSetPoint = 0.3;

		public static final double kFinalGearRatio = kMotorGearRatio * kSprocketRatio;
		public static final double kSprocketCircumference = Math.PI * (kSprocketRadius * 2);
		public static final double kFinalConversionFactor = kFinalGearRatio * kSprocketCircumference;

		public static final double kP = 2.5;
		public static final double kI = 0.0;
		public static final double kD = 0.0;

	}

	public static final class ManipulatorConstants {
		public static final int kIdlePosition = 5;
		public static final int kTopNodePosition = 1;
		public static final int kMiddleNodePosition = 2;
		public static final int kFrontGroundPosition = 3;
		public static final int kRearGroundPosition = 4;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kCoDriverControllerPort = 1;
		public static final double kDeadband = 0.05;
	}
}