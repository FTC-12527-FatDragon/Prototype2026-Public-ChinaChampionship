package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Vision extends SubsystemBase {
    private Limelight3A limelight;

    public Vision(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.start();
    }

    /**
     * @return robot position in the field (can be null if the limelight doesn't see anything)
     */
    public Pose3D getRobotPosition(){
        List<FiducialResult> fiducialResult = limelight.getLatestResult().getFiducialResults();
        if (!fiducialResult.isEmpty()
                && fiducialResult.get(0).getTargetArea() >= VisionConstants.targetArea) {
            return fiducialResult.get(0).getRobotPoseFieldSpace();
        }
        return null;
    }
}
