package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeTransfer {
    private Intake2 motor1;

    public IntakeTransfer(HardwareMap hardwareMap) {
        motor1 = new Intake2(hardwareMap);
    }

    public enum IntakeTransferState {
        IDLE,
        INTAKING,
        TRANSFERRING,
        OUTTAKE
    } IntakeTransferState intakeTransferState;

    public void updateTransfer() {
        switch (intakeTransferState) {
            case IDLE:
                break;
            case INTAKING:
                break;
            case TRANSFERRING:
                break;
            case OUTTAKE:
                break;
        }
    }
}
