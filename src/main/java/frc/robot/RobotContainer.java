package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Joystick Axes */
    private final int leftThumbXID = XboxController.Axis.kLeftX.value;
    private final int leftThumbYID = XboxController.Axis.kLeftY.value;
    private final int rightThumbXID = XboxController.Axis.kRightX.value;

    private final int leftTriggerID = XboxController.Axis.kLeftTrigger.value;
    private final int rightTriggerID = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton resetButton = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();

    private Command resetCommand = new Reset(s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(leftThumbYID), // translation axis
                () -> -driver.getRawAxis(leftThumbXID), // strafe axis
                () -> -driver.getRawAxis(rightThumbXID),  // rotation axis
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Shooter.setDefaultCommand(
            new Shoot(
                s_Shooter, 
                () -> driver.getRawAxis(leftTriggerID),
                () -> driver.getRawAxis(rightTriggerID)
            )
        );

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@c
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        resetButton.onTrue(resetCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
