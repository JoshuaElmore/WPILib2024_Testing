// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AutoCommandChooser;
import frc.robot.utility.RobotIdentity;
import frc.robot.utility.SubsystemFactory;

public class RobotContainer {

  // Creating Controlers
  @SuppressWarnings({ "unused" })
  private final CommandXboxController driveController = new CommandXboxController(DRIVE_CONTROLLER_PORT);
  @SuppressWarnings({ "unused" })
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  @SuppressWarnings({ "unused" })
  private final CommandXboxController testController = new CommandXboxController(TEST_CONTROLLER_PORT);

  private RobotIdentity identity;

  private Drive driveSubsystem;

  private DefaultDriveCommand defaultDriveCommand;

  private AutoCommandChooser autoChooser;

  public RobotContainer() {
    identity = RobotIdentity.getIdentity();

    createSubsystems(); // Create our subsystems.
    createCommands(); // Create our commands
    configureButtonBindings(); // Configure the button bindings
    setupAutoChooser(); // Setup the auto chooser
  }

  private void createSubsystems() {
    driveSubsystem = SubsystemFactory.createDriveTrain(identity);

    AutoCommandFactory.init(driveSubsystem);

  }

  private void createCommands() {
    defaultDriveCommand = new DefaultDriveCommand(driveSubsystem,
        () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
        () -> -driveController.getLeftX());
    driveSubsystem.setDefaultCommand(defaultDriveCommand);
  }

  private void configureButtonBindings() {
    driveController.start().onTrue(new InstantCommand(() -> driveSubsystem.resetPose()));
    driveController.back().onTrue(new InstantCommand(() -> driveSubsystem.resetGyro()));
  }

  private void setupAutoChooser() {
    autoChooser = new AutoCommandChooser();

    // Register all the supported auto commands
    autoChooser.registerDefaultCreator("Do Nothing", null);

    // Test auto commands that we only register with the chooser if we are not running in competition
    if (!Constants.COMPETITION_MODE) {
        autoChooser.registerCreator("Test", () -> new PathPlannerAuto("TestAuto"));
    }

    // Setup the chooser in shuffleboard
    autoChooser.setup("Driver", 0, 0, 3, 1);
}

  public Command getAutonomousCommand() {
    return autoChooser.getAutonomousCommand();
  }
}
