package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlMap;
import frc.robot.commands.IntakeCommands;
import frc.robot.util.StateMachine;

public class Intake extends SubsystemBase {
    private IntakeIOTalonFX io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final String key = "Intake";
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }
        return instance;
    }

    public StateMachine createIntakeStateMachine() {
        // State machine object should exist for duration of match
        StateMachine intakeFSM = new StateMachine("intakeFSM");
        
        // Define commands for states
        Command stopCommand = IntakeCommands.runVoltageCommand(getInstance(), 0.0);
        Command startCommand = IntakeCommands.runVoltageCommand(getInstance(), IntakeConstants.fastIntakeVoltage);
        Command slowCommand = IntakeCommands.runVoltageCommand(getInstance(), IntakeConstants.slowIntakeVoltage);
        Command reverseCommand = IntakeCommands.runVoltageCommand(getInstance(), IntakeConstants.reverseIntakeVoltage);

        // Create states
        StateMachine.State stopState = intakeFSM.addState("stopState", stopCommand);
        StateMachine.State fastState = intakeFSM.addState("fastState", startCommand);
        StateMachine.State slowState = intakeFSM.addState("slowState", slowCommand);
        StateMachine.State reverseState = intakeFSM.addState("reverseState", reverseCommand);

        intakeFSM.setInitialState(stopState); // On init, stop intake

        stopState.switchTo(fastState).when(() -> ControlMap.getInstance().intakeJustPressed());
        fastState.switchTo(stopState).when(() -> ControlMap.getInstance().intakeJustPressed());

        intakeFSM.switchFromAny(stopState, fastState).to(reverseState).when(() -> 
            ControlMap.getInstance().reverseButtonHeld());
    
        return intakeFSM;
    }

    public Intake() {
        io = new IntakeIOTalonFX();
    }

    public void runVoltage(double volts) {
        Logger.recordOutput(key + "/TargetVoltage", volts);
        io.runVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(key, inputs);
    }
}
