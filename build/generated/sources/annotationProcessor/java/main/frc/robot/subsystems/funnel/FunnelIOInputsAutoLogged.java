package frc.robot.subsystems.funnel;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FunnelIOInputsAutoLogged extends FunnelIO.FunnelIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("FunnelMotorConnected", funnelMotorConnected);
    table.put("FunnelPositionRot", funnelPositionRot);
    table.put("FunnelVelocityRPS", funnelVelocityRPS);
    table.put("FunnelAppliedVolts", funnelAppliedVolts);
    table.put("FunnelStatorCurrentAmps", funnelStatorCurrentAmps);
    table.put("FunnelSupplyCurrentAmps", funnelSupplyCurrentAmps);
    table.put("IndexerMotorConnected", indexerMotorConnected);
    table.put("IndexerPositionRot", indexerPositionRot);
    table.put("IndexerVelocityRPS", indexerVelocityRPS);
    table.put("IndexerAppliedVolts", indexerAppliedVolts);
    table.put("IndexerStatorCurrentAmps", indexerStatorCurrentAmps);
    table.put("IndexerSupplyCurrentAmps", indexerSupplyCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    funnelMotorConnected = table.get("FunnelMotorConnected", funnelMotorConnected);
    funnelPositionRot = table.get("FunnelPositionRot", funnelPositionRot);
    funnelVelocityRPS = table.get("FunnelVelocityRPS", funnelVelocityRPS);
    funnelAppliedVolts = table.get("FunnelAppliedVolts", funnelAppliedVolts);
    funnelStatorCurrentAmps = table.get("FunnelStatorCurrentAmps", funnelStatorCurrentAmps);
    funnelSupplyCurrentAmps = table.get("FunnelSupplyCurrentAmps", funnelSupplyCurrentAmps);
    indexerMotorConnected = table.get("IndexerMotorConnected", indexerMotorConnected);
    indexerPositionRot = table.get("IndexerPositionRot", indexerPositionRot);
    indexerVelocityRPS = table.get("IndexerVelocityRPS", indexerVelocityRPS);
    indexerAppliedVolts = table.get("IndexerAppliedVolts", indexerAppliedVolts);
    indexerStatorCurrentAmps = table.get("IndexerStatorCurrentAmps", indexerStatorCurrentAmps);
    indexerSupplyCurrentAmps = table.get("IndexerSupplyCurrentAmps", indexerSupplyCurrentAmps);
  }

  public FunnelIOInputsAutoLogged clone() {
    FunnelIOInputsAutoLogged copy = new FunnelIOInputsAutoLogged();
    copy.funnelMotorConnected = this.funnelMotorConnected;
    copy.funnelPositionRot = this.funnelPositionRot;
    copy.funnelVelocityRPS = this.funnelVelocityRPS;
    copy.funnelAppliedVolts = this.funnelAppliedVolts;
    copy.funnelStatorCurrentAmps = this.funnelStatorCurrentAmps;
    copy.funnelSupplyCurrentAmps = this.funnelSupplyCurrentAmps;
    copy.indexerMotorConnected = this.indexerMotorConnected;
    copy.indexerPositionRot = this.indexerPositionRot;
    copy.indexerVelocityRPS = this.indexerVelocityRPS;
    copy.indexerAppliedVolts = this.indexerAppliedVolts;
    copy.indexerStatorCurrentAmps = this.indexerStatorCurrentAmps;
    copy.indexerSupplyCurrentAmps = this.indexerSupplyCurrentAmps;
    return copy;
  }
}
