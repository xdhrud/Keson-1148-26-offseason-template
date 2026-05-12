package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakeMotorMasterConnected", intakeMotorMasterConnected);
    table.put("IntakeMotorFollowerConnected", intakeMotorFollowerConnected);
    table.put("IntakeVelocityRPS", intakeVelocityRPS);
    table.put("IntakeAppliedVolts", intakeAppliedVolts);
    table.put("IntakeStatorCurrentAmps", intakeStatorCurrentAmps);
    table.put("IntakeSupplyCurrentAmps", intakeSupplyCurrentAmps);
    table.put("IntakeTempC", intakeTempC);
  }

  @Override
  public void fromLog(LogTable table) {
    intakeMotorMasterConnected = table.get("IntakeMotorMasterConnected", intakeMotorMasterConnected);
    intakeMotorFollowerConnected = table.get("IntakeMotorFollowerConnected", intakeMotorFollowerConnected);
    intakeVelocityRPS = table.get("IntakeVelocityRPS", intakeVelocityRPS);
    intakeAppliedVolts = table.get("IntakeAppliedVolts", intakeAppliedVolts);
    intakeStatorCurrentAmps = table.get("IntakeStatorCurrentAmps", intakeStatorCurrentAmps);
    intakeSupplyCurrentAmps = table.get("IntakeSupplyCurrentAmps", intakeSupplyCurrentAmps);
    intakeTempC = table.get("IntakeTempC", intakeTempC);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.intakeMotorMasterConnected = this.intakeMotorMasterConnected;
    copy.intakeMotorFollowerConnected = this.intakeMotorFollowerConnected;
    copy.intakeVelocityRPS = this.intakeVelocityRPS;
    copy.intakeAppliedVolts = this.intakeAppliedVolts;
    copy.intakeStatorCurrentAmps = this.intakeStatorCurrentAmps;
    copy.intakeSupplyCurrentAmps = this.intakeSupplyCurrentAmps;
    copy.intakeTempC = this.intakeTempC;
    return copy;
  }
}
