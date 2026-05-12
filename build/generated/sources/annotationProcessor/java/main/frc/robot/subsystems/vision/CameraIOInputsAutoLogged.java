package frc.robot.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CameraIOInputsAutoLogged extends CameraIO.CameraIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TagCount", tagCount);
    table.put("Ambiguity", ambiguity);
    table.put("DistToCamera", distToCamera);
    table.put("EstimatedPose", estimatedPose);
  }

  @Override
  public void fromLog(LogTable table) {
    tagCount = table.get("TagCount", tagCount);
    ambiguity = table.get("Ambiguity", ambiguity);
    distToCamera = table.get("DistToCamera", distToCamera);
    estimatedPose = table.get("EstimatedPose", estimatedPose);
  }

  public CameraIOInputsAutoLogged clone() {
    CameraIOInputsAutoLogged copy = new CameraIOInputsAutoLogged();
    copy.tagCount = this.tagCount;
    copy.ambiguity = this.ambiguity;
    copy.distToCamera = this.distToCamera;
    copy.estimatedPose = this.estimatedPose;
    return copy;
  }
}
