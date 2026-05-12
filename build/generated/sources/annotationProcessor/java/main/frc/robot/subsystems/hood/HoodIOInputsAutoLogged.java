package frc.robot.subsystems.hood;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodIOInputsAutoLogged extends HoodIO.HoodIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("HoodMotorConnected", hoodMotorConnected);
    table.put("HoodPositionRot", hoodPositionRot);
    table.put("HoodPositionDegrees", hoodPositionDegrees);
    table.put("HoodVelocityDPS", hoodVelocityDPS);
    table.put("HoodAppliedVolts", hoodAppliedVolts);
    table.put("HoodStatorCurrentAmps", hoodStatorCurrentAmps);
    table.put("HoodSupplyCurrentAmps", hoodSupplyCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    hoodMotorConnected = table.get("HoodMotorConnected", hoodMotorConnected);
    hoodPositionRot = table.get("HoodPositionRot", hoodPositionRot);
    hoodPositionDegrees = table.get("HoodPositionDegrees", hoodPositionDegrees);
    hoodVelocityDPS = table.get("HoodVelocityDPS", hoodVelocityDPS);
    hoodAppliedVolts = table.get("HoodAppliedVolts", hoodAppliedVolts);
    hoodStatorCurrentAmps = table.get("HoodStatorCurrentAmps", hoodStatorCurrentAmps);
    hoodSupplyCurrentAmps = table.get("HoodSupplyCurrentAmps", hoodSupplyCurrentAmps);
  }

  public HoodIOInputsAutoLogged clone() {
    HoodIOInputsAutoLogged copy = new HoodIOInputsAutoLogged();
    copy.hoodMotorConnected = this.hoodMotorConnected;
    copy.hoodPositionRot = this.hoodPositionRot;
    copy.hoodPositionDegrees = this.hoodPositionDegrees;
    copy.hoodVelocityDPS = this.hoodVelocityDPS;
    copy.hoodAppliedVolts = this.hoodAppliedVolts;
    copy.hoodStatorCurrentAmps = this.hoodStatorCurrentAmps;
    copy.hoodSupplyCurrentAmps = this.hoodSupplyCurrentAmps;
    return copy;
  }
}
