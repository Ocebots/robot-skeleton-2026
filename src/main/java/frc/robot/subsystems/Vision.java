package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    public static final PhotonCamera leftApriltagCamera = new PhotonCamera("leftApril");
    public static final PhotonCamera rightApriltagCamera = new PhotonCamera("rightApril");
}
