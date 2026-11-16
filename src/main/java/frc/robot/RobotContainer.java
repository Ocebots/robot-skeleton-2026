// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.config.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.util.Units;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public static Pigeon2 pigeon2 = new Pigeon2(1); // pigeon can id
  Translation2d m_frontLeftLocation =
      new Translation2d(Units.inchesToMeters(10.875), Units.inchesToMeters(10.875));
  Translation2d m_frontRightLocation =
      new Translation2d(Units.inchesToMeters(10.875), Units.inchesToMeters(-10.875));
  Translation2d m_backLeftLocation =
      new Translation2d(Units.inchesToMeters(-10.875), Units.inchesToMeters(10.875));
  Translation2d m_backRightLocation =
      new Translation2d(Units.inchesToMeters(-10.855), Units.inchesToMeters(-10.875));
  public SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          new SwerveDriveKinematics(
              m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation),
          pigeon2.getRotation2d(),
          new SwerveModulePosition[] {
            drivetrain.getModule(0).getPosition(true),
            drivetrain.getModule(1).getPosition(true),
            drivetrain.getModule(2).getPosition(true),
            drivetrain.getModule(3).getPosition(true)
          },
          new Pose2d(0.0, 0.0, new Rotation2d()));
  static Field2d m_field = new Field2d();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
}
