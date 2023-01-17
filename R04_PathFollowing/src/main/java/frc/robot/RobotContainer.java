// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.List;

import static frc.robot.commands.ChassisPoseListCommand.makeWaypoint;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure ofthe robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Chassis chassis = new Chassis();

  private final XboxController xboxController = new XboxController(0);

  private Command autonomousCommand = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    chassis.setDefaultCommand(new ChassisDriveCommand(
        () -> xboxController.getRawAxis(1),
        () -> xboxController.getRawAxis(0),
        chassis));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // autonomousCommand = new ChassisPathWeaverCommand("eight.wpilib.json",
    // chassis);

    // autonomousCommand = new ChassisWaypointCommand(List.of(
    // new Translation2d(0.5, 0.25),
    // new Translation2d(1.0, 0),
    // new Translation2d(0.5, -0.25)), chassis);

    // autonomousCommand = new ChassisWaypointCommand(
    // new Pose2d(0, 0, new Rotation2d(0)),
    // List.of(
    // new Translation2d(0.7, 0.3),
    // new Translation2d(1.3, -0.3)),
    // new Pose2d(2.0, 0.0, new Rotation2d(0.0)),
    // chassis);

    autonomousCommand = new ChassisPoseListCommand(
        List.of(
            makeWaypoint(0.7, 0.3, 0),
            makeWaypoint(1.3, -0.3, 0),
            makeWaypoint(1.5, 0.0, 90)),
        chassis);

    return autonomousCommand
        .andThen(() -> chassis.stop());
  }
}
