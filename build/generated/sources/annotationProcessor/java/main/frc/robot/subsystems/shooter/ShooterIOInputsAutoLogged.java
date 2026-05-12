package frc.robot.subsystems.shooter;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ShooterMotorMasterConnected", shooterMotorMasterConnected);
    table.put("ShooterMotorFollower1Connected", shooterMotorFollower1Connected);
    table.put("ShooterMotorFollower2Connected", shooterMotorFollower2Connected);
    table.put("ShooterMotorFollower3Connected", shooterMotorFollower3Connected);
    table.put("ShooterVelocityRPS", shooterVelocityRPS);
    table.put("ShooterAppliedVolts", shooterAppliedVolts);
    table.put("ShooterStatorCurrentAmps", shooterStatorCurrentAmps);
    table.put("ShooterSupplyCurrentAmps", shooterSupplyCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    shooterMotorMasterConnected = table.get("ShooterMotorMasterConnected", shooterMotorMasterConnected);
    shooterMotorFollower1Connected = table.get("ShooterMotorFollower1Connected", shooterMotorFollower1Connected);
    shooterMotorFollower2Connected = table.get("ShooterMotorFollower2Connected", shooterMotorFollower2Connected);
    shooterMotorFollower3Connected = table.get("ShooterMotorFollower3Connected", shooterMotorFollower3Connected);
    shooterVelocityRPS = table.get("ShooterVelocityRPS", shooterVelocityRPS);
    shooterAppliedVolts = table.get("ShooterAppliedVolts", shooterAppliedVolts);
    shooterStatorCurrentAmps = table.get("ShooterStatorCurrentAmps", shooterStatorCurrentAmps);
    shooterSupplyCurrentAmps = table.get("ShooterSupplyCurrentAmps", shooterSupplyCurrentAmps);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.shooterMotorMasterConnected = this.shooterMotorMasterConnected;
    copy.shooterMotorFollower1Connected = this.shooterMotorFollower1Connected;
    copy.shooterMotorFollower2Connected = this.shooterMotorFollower2Connected;
    copy.shooterMotorFollower3Connected = this.shooterMotorFollower3Connected;
    copy.shooterVelocityRPS = this.shooterVelocityRPS;
    copy.shooterAppliedVolts = this.shooterAppliedVolts;
    copy.shooterStatorCurrentAmps = this.shooterStatorCurrentAmps;
    copy.shooterSupplyCurrentAmps = this.shooterSupplyCurrentAmps;
    return copy;
  }
}
