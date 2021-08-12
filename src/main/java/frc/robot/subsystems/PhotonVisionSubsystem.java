package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import java.util.OptionalDouble;

import static frc.robot.Constants.PhotonVisionConstants.*;
import static frc.robot.Ports.PhotonVisionPorts.PISTON_PORT;

/**
 * This subsystem controls the Camera and LED.
 * Axis are x (positive-right), y (positive-up).
 */
public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera = new PhotonCamera("GalaxiaCam");
    private final Solenoid piston = new Solenoid(PISTON_PORT);

    private final static PhotonVisionSubsystem INSTANCE = new PhotonVisionSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static PhotonVisionSubsystem getInstance() {
        return INSTANCE;
    }

    private PhotonVisionSubsystem() {
    }

    public void toggleLEDOnOff() {
        if (photonCamera.getLEDMode() == LEDMode.kOn) {
            photonCamera.setLED(LEDMode.kOff);
            return;
        }
        photonCamera.setLED(LEDMode.kOn);
    }

    public void setLedMode(LEDMode ledMode) {
        photonCamera.setLED(ledMode);
    }

    public OptionalDouble getTargetXError() {
        if (!photonCamera.hasTargets()) return OptionalDouble.empty();

        return OptionalDouble.of(Math.toDegrees(-photonCamera.getLatestResult().getBestTarget().getYaw()));
    }

    public OptionalDouble getTargetYError() {
        if (!photonCamera.hasTargets()) return OptionalDouble.empty();

        return OptionalDouble.of(Math.toDegrees(photonCamera.getLatestResult().getBestTarget().getPitch()));
    }

    public OptionalDouble getDistanceFromTarget() {
        if (!photonCamera.hasTargets()) return OptionalDouble.empty();

        return OptionalDouble.of(PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT,
                TARGET_HEIGHT,
                Math.toRadians(getCurrentCameraPitch()),
                photonCamera.getLatestResult().getBestTarget().getPitch()));
    }

    public enum PistonMode {
        SHORT_RANGE(true), LONG_RANGE(false);

        boolean value;

        PistonMode(boolean value) {
            this.value = value;
        }
    }

    public PistonMode getPistonMode() {
        if (piston.get() == PistonMode.SHORT_RANGE.value) return PistonMode.SHORT_RANGE;
        return PistonMode.LONG_RANGE;
    }

    public void setPistonMode(PistonMode pistonMode) {
        piston.set(pistonMode.value);
    }

    public void togglePistonMode() {
        piston.toggle();
    }

    public double getCurrentCameraPitch() {
        if (getPistonMode() == PistonMode.SHORT_RANGE) return SHORT_RANGE_CAMERA_PITCH;
        return LONG_RANGE_CAMERA_PITCH;
    }

    public void updatePipelineIndex() {
        if (getPistonMode() == PistonMode.SHORT_RANGE) photonCamera.setPipelineIndex(SHORT_RANGE_PIPELINE_INDEX);
        if (getPistonMode() == PistonMode.LONG_RANGE) photonCamera.setPipelineIndex(LONG_RANGE_PIPELINE_INDEX);
    }

    @Override
    public void periodic() {
        updatePipelineIndex();
    }
}
