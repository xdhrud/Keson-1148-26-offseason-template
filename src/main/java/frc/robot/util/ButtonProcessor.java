package frc.robot.util;

public class ButtonProcessor {

    public ButtonProcessor() {}

    // Debounce processor
    private boolean wasPressed = false;
    private boolean debounced = false;

    public void checkDebounce(boolean pressInput) {
        debounced = pressInput && !wasPressed;
        wasPressed = pressInput;
    }
    
    public boolean justPressed() {
        return debounced;
    }
    
    // Hold processor
    private boolean isHolding = false;

    public void checkHold(boolean pressInput) {
        if (pressInput) isHolding = true;
        else isHolding = false;
    }

    public boolean isHolding() {
        return isHolding;
    }
}
