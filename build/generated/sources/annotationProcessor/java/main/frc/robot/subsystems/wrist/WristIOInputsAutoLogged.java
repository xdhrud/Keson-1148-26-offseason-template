package frc.robot.subsystems.wrist;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class WristIOInputsAutoLogged extends WristIO.WristIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("WristMotorConnected", wristMotorConnected);
    table.put("WristPositionRot", wristPositionRot);
    table.put("WristVelocityRPS", wristVelocityRPS);
    table.put("WristAppliedVolts", wristAppliedVolts);
    table.put("WristStatorCurrentAmps", wristStatorCurrentAmps);
    table.put("WristSupplyCurrentAmps", wristSupplyCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    wristMotorConnected = table.get("WristMotorConnected", wristMotorConnected);
    wristPositionRot = table.get("WristPositionRot", wristPositionRot);
    wristVelocityRPS = table.get("WristVelocityRPS", wristVelocityRPS);
    wristAppliedVolts = table.get("WristAppliedVolts", wristAppliedVolts);
    wristStatorCurrentAmps = table.get("WristStatorCurrentAmps", wristStatorCurrentAmps);
    wristSupplyCurrentAmps = table.get("WristSupplyCurrentAmps", wristSupplyCurrentAmps);
  }

  public WristIOInputsAutoLogged clone() {
    WristIOInputsAutoLogged copy = new WristIOInputsAutoLogged();
    copy.wristMotorConnected = this.wristMotorConnected;
    copy.wristPositionRot = this.wristPositionRot;
    copy.wristVelocityRPS = this.wristVelocityRPS;
    copy.wristAppliedVolts = this.wristAppliedVolts;
    copy.wristStatorCurrentAmps = this.wristStatorCurrentAmps;
    copy.wristSupplyCurrentAmps = this.wristSupplyCurrentAmps;
    return copy;
  }
}
