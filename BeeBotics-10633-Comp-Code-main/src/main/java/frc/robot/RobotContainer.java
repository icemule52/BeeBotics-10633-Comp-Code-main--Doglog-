
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveArmToRotationCommand;
import frc.robot.commands.MoveElevatorToPositionCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class RobotContainer {
    // The robot's subsystems
    private elevator elevator;
    private Arm arm;
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final elevator m_elevator = new elevator();
    private final Arm m_arm = new Arm();
    private final SendableChooser<Command> autoChooser;

    // Controllers
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
   
    SmartDashboard.putNumber("Position", elevator.getHeight());
    SmartDashboard.putNumber("Rotation", arm.getRotation());
    DogLog.log("Debug/SwerveState", new SwerveModuleState());
    DogLog.log("Debug/Position", elevator.getHeight());
    DogLog.log("Debug/Rotation", arm.getRotation());
    DogLog.log("Debug/Json", "{\"test\": \"json\"}", "json");

        NamedCommands.registerCommand("Place L4",
                new MoveElevatorToPositionCommand(m_elevator, -1).alongWith(
                        new MoveArmToRotationCommand(m_arm, -0.021)));
        NamedCommands.registerCommand("ReHome",
                new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                        new MoveArmToRotationCommand(m_arm, 0)));
        NamedCommands.registerCommand("Yoink",
                new MoveElevatorToPositionCommand(m_elevator, 0.5).alongWith(
                        new MoveArmToRotationCommand(m_arm, 0)));

        configureButtonBindings();
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withCaptureNt(true));
    DogLog.setPdh(new PowerDistribution());
        // Configure default commands
        m_robotDrive.setDefaultCommand(

                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),
                                true),
                        m_robotDrive));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // SmartDashboard.putData("Put 0",new MoveelevatorToPositionCommand(m_Elevator,
        // 0));
        // SmartDashboard.putData("Put 1",new MoveelevatorToPositionCommand(m_Elevator,
        // 1));

        // SmartDashboard.putData("Put 2",new MoveArmToRotationCommand(m_arm, 0));
        // SmartDashboard.putData("Put 3",new MoveArmToRotationCommand(m_arm, -0.023));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));
        // Climb
        new JoystickButton(m_driverController, Button.kL1.value)
                        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));

        // L4
        m_operatorController.x().whileTrue(
                new MoveElevatorToPositionCommand(m_elevator, -1).alongWith(
                        new MoveArmToRotationCommand(m_arm, -0.02))) // Changed from comp # of -0.021
                .whileFalse(
                        new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                                new MoveArmToRotationCommand(m_arm, 0)));
        // L3
        m_operatorController.b().whileTrue(
                new MoveElevatorToPositionCommand(m_elevator, -0.05).alongWith(
                        new MoveArmToRotationCommand(m_arm, -0.0215))) // Changed from comp # of -0.022
                .whileFalse(
                        new MoveArmToRotationCommand(m_arm, 0).andThen(
                                new MoveElevatorToPositionCommand(m_elevator, 0)));
        // L2
        m_operatorController.y().whileTrue(
                new MoveElevatorToPositionCommand(m_elevator, 0.15).alongWith(
                        new MoveArmToRotationCommand(m_arm, -0.017))) // Changed from comp # of -0.018
                .whileFalse(
                        new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                                new MoveArmToRotationCommand(m_arm, 0)));
        // L1
        m_operatorController.rightTrigger().whileTrue(
                new MoveArmToRotationCommand(m_arm, -0.015).andThen(
                        new MoveElevatorToPositionCommand(m_elevator, 0.3)))
                .whileFalse(
                        new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                                new MoveArmToRotationCommand(m_arm, 0)));
        // Yoink
        m_operatorController.a().whileTrue(
                new MoveElevatorToPositionCommand(m_elevator, 0.6).alongWith(
                        new MoveArmToRotationCommand(m_arm, 0)))
                .whileFalse(
                        new MoveElevatorToPositionCommand(m_elevator, 0).alongWith(
                                new MoveArmToRotationCommand(m_arm, 0)));
        // Reset Arm
        m_operatorController.rightBumper().whileTrue(
                new MoveArmToRotationCommand(m_arm, 0.005)).whileFalse(
                        new InstantCommand(() -> m_arm.resetRotation()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return m_robotDrive.getAutonomousCommand("Auto1"); //put the name of the AUTO
        // that you want here
        // Return the autonomous command
        return autoChooser.getSelected();
    }
    public void resetGyro(double angle) {
        m_robotDrive.setGyro(angle);
    }
}
