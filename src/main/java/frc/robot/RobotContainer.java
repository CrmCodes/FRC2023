package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCmd.ArmSetPosCmd;
import frc.robot.commands.AutoCmd.ElevatorHasResetCmd;
import frc.robot.commands.AutoCmd.ElevatorSetPosCmd;
import frc.robot.commands.AutoCmd.ExtenderSetPowerCmd;
import frc.robot.commands.AutoCmd.ExtenderToHomeCmd;
import frc.robot.commands.AutoCmd.LimelightAlignTapeCmd;
import frc.robot.commands.AutoCmd.PPSwerveTrajCmd;
import frc.robot.commands.AutoCmd.intakeSetAutoCmd;
import frc.robot.commands.AutoCmd.AutoSequenceCmd.FirstPieceAuto;
import frc.robot.commands.TeleOpCmd.ArmJoystickCmd;
import frc.robot.commands.TeleOpCmd.ArmPositionCmd;
import frc.robot.commands.TeleOpCmd.ElevatorJoystickCmd;
import frc.robot.commands.TeleOpCmd.ExtenderJoystickCmd;
import frc.robot.commands.TeleOpCmd.SwerveTeleOpCmd;
import frc.robot.commands.TeleOpCmd.intakeSetCmd;
import frc.robot.subsystems.ColorDetectCVSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final PIDElevatorSubsystem elevatorSubsystem = new PIDElevatorSubsystem();
	public final PIDArmSubsystem armSubsystem = new PIDArmSubsystem();
	public final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
	public final ColorDetectCVSubsystem colorDetectCVSubsystem = new ColorDetectCVSubsystem();

	private final XboxController joystick_1 = new XboxController(OIConstants.kDriverControllerPort);
	private final XboxController joystick_2 = new XboxController(OIConstants.kCoDriverControllerPort);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		driveSubsystem.setDefaultCommand(new SwerveTeleOpCmd(
			driveSubsystem, 
			() -> -joystick_1.getLeftY(), 
			() -> -joystick_1.getLeftX(),
			() -> -joystick_1.getRightX(), 
			() -> joystick_1.getRightBumper(),
			() -> joystick_1.getRightTriggerAxis()));

		elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, () -> joystick_2.getRightY()));
		extenderSubsystem.setDefaultCommand(new ExtenderJoystickCmd(extenderSubsystem, () -> -joystick_2.getPOV()));

		// armSubsystem.setDefaultCommand(new ArmJoystickCmd(armSubsystem, joystick_2));


		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(joystick_1, XboxController.Button.kA.value).whileTrue(Commands.runOnce(() -> driveSubsystem.zeroHeading(), driveSubsystem));
		// new JoystickButton(joystick_2, XboxController.Button.kA.value).whileTrue(Commands.runOnce(() -> armSubsystem.setPos(40), armSubsystem));
		// new JoystickButton(joystick_2, XboxController.Button.kB.value).whileTrue(Commands.runOnce(() -> armSubsystem.setPos(90), armSubsystem));
		// new JoystickButton(joystick_2, XboxController.Button.kX.value).whileTrue(Commands.runOnce(() -> armSubsystem.setPos(-90), armSubsystem));
		new JoystickButton(joystick_2, XboxController.Button.kA.value).whileTrue(new ArmPositionCmd(armSubsystem, ManipulatorConstants.kFrontGroundPosition, () -> -joystick_2.getLeftY()));
		new JoystickButton(joystick_2, XboxController.Button.kB.value).whileTrue(new ArmPositionCmd(armSubsystem, ManipulatorConstants.kTopNodePosition, () -> -joystick_2.getLeftY()));
		new JoystickButton(joystick_2, XboxController.Button.kX.value).whileTrue(new ArmPositionCmd(armSubsystem, ManipulatorConstants.kRearGroundPosition, () -> -joystick_2.getLeftY()));
		new JoystickButton(joystick_2, XboxController.Button.kY.value).whileTrue(new ArmPositionCmd(armSubsystem, ManipulatorConstants.kIdlePosition, () -> -joystick_2.getLeftY()));
		new JoystickButton(joystick_2, XboxController.Button.kLeftBumper.value).whileTrue(new intakeSetCmd(intakeSubsystem, 1.0));
		new JoystickButton(joystick_2, XboxController.Button.kRightBumper.value).whileTrue(new intakeSetCmd(intakeSubsystem, -1.0));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		PathPlannerTrajectory PPfirstTrajectory = PathPlanner.loadPath("First Piece Path", new PathConstraints(2, 3));
		PathPlannerTrajectory PPsecondTrajectory = PathPlanner.loadPath("Move Back Path", new PathConstraints(2, 3));
		PathPlannerTrajectory PPthirdTrajectory = PathPlanner.loadPath("To Second Piece Path", new PathConstraints(3, 3));
		PathPlannerTrajectory PPfourthTrajectory = PathPlanner.loadPath("Place Second Piece Path", new PathConstraints(3, 3));
		PPSwerveTrajCmd ppSwerveTrajCmd = new PPSwerveTrajCmd();
	

		return new SequentialCommandGroup(
			new FirstPieceAuto(driveSubsystem, elevatorSubsystem, armSubsystem, extenderSubsystem, intakeSubsystem),
			new ExtenderToHomeCmd(extenderSubsystem),
			new ArmSetPosCmd(armSubsystem, 0),
			new ElevatorSetPosCmd(elevatorSubsystem, -0.05),
			// new ParallelCommandGroup(
			// 	new SequentialCommandGroup(
			// 		new ExtenderToHomeCmd(extenderSubsystem),
			// 		new ArmSetPosCmd(armSubsystem, -60),
			// 		new WaitCommand(2),
			// 		new ParallelDeadlineGroup(new WaitCommand(.8), new ExtenderSetPowerCmd(extenderSubsystem, 1.0))),
			// 	ppSwerveTrajCmd.PPSwerveTrajCommand(driveSubsystem, PPthirdTrajectory)),
			// new InstantCommand(() -> driveSubsystem.stopModules()),
			// new ParallelCommandGroup(
			// 	new ElevatorSetPosCmd(elevatorSubsystem, 0.6),
			// 	new intakeSetAutoCmd(intakeSubsystem, -1.0)),
			// new ParallelCommandGroup(
			// 	new ParallelCommandGroup(
			// 		new intakeSetAutoCmd(intakeSubsystem, 0),
			// 		new ExtenderToHomeCmd(extenderSubsystem),
			// 		new ElevatorSetPosCmd(elevatorSubsystem, -0.05)).andThen(
			// 			new ArmSetPosCmd(armSubsystem, 90)),
			// 	ppSwerveTrajCmd.PPSwerveTrajCommand(driveSubsystem, PPfourthTrajectory)),
			// new InstantCommand(() -> driveSubsystem.stopModules()),
			// new ParallelDeadlineGroup(new WaitCommand(.8), new ExtenderSetPowerCmd(extenderSubsystem, 1.0)),
			// new ExtenderSetPowerCmd(extenderSubsystem, 0),
			// new ParallelDeadlineGroup(new WaitCommand(.5), new intakeSetAutoCmd(intakeSubsystem, 1.0)),
			// new intakeSetAutoCmd(intakeSubsystem, 0),
			new PrintCommand("AUTO ENDED")
		);

		// return new SequentialCommandGroup(
		// 	new ElevatorHasResetCmd(elevatorSubsystem),
		// 	new ElevatorSetPosCmd(elevatorSubsystem, -0.05),
		// 	new ArmSetPosCmd(armSubsystem, 50),
		// 	new WaitCommand(1),
		// 	new ArmSetPosCmd(armSubsystem, -60),
		// 	new WaitCommand(3),
		// 	new ParallelDeadlineGroup(new WaitCommand(.8), new ExtenderSetPowerCmd(extenderSubsystem, 1.0)),
		// 	new ParallelCommandGroup(
		// 		new ElevatorSetPosCmd(elevatorSubsystem, 0.6),
		// 		new intakeSetAutoCmd(intakeSubsystem, -1.0)),
		// 	new WaitCommand(1),
		// 	new ParallelCommandGroup(
		// 		new intakeSetAutoCmd(intakeSubsystem, 0),
		// 		new ExtenderToHomeCmd(extenderSubsystem),
		// 		new ElevatorSetPosCmd(elevatorSubsystem, -0.05)),
		// 	new ArmSetPosCmd(armSubsystem, 0),
		// 	new PrintCommand("AUTO ENDED")

		// );
	}
}
