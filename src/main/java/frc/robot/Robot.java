// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.VisionConfig;
import frc.robot.subsystems.Vision;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final Field2d field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    var currentPose = m_robotContainer.drivetrain.getState().Pose;
    VisionConfig.photonPoseEstimatorLeft.setReferencePose(currentPose);
    VisionConfig.photonPoseEstimatorRight.setReferencePose(currentPose);

    processCamera(Vision.leftApriltagCamera, VisionConfig.photonPoseEstimatorLeft, currentPose);
    processCamera(Vision.rightApriltagCamera, VisionConfig.photonPoseEstimatorRight, currentPose);

    field.setRobotPose(m_robotContainer.m_odometry.getEstimatedPosition());
  }

  private void processCamera(
      PhotonCamera camera, PhotonPoseEstimator estimator, Pose2d currentPose) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      PhotonPipelineResult latestResult = results.get(results.size() - 1);
      estimator
          .update(latestResult)
          .ifPresent(
              (pose) -> {
                if (Vision.isValidPose(pose, currentPose)) {
                  var stdDevs = Vision.getDynamicStdDevs(pose);
                  m_robotContainer.drivetrain.addVisionMeasurement(
                      pose.estimatedPose.toPose2d(), pose.timestampSeconds, stdDevs);
                }
              });
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
