package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class compTeleop extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    enum ArmPos{
        HOME,
        BACK,
        OUT
    }

    enum ArmMove{
        H2O,
        H2B,
        O2H,
        B2H,
        O2B
    }

    ArmPos pos = ArmPos.HOME;
    ArmPos prevPos = pos;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a || gamepad2.a){
            pos = ArmPos.HOME;
        }


        switch(pos){
            case HOME:

                break;


        }

    }
}
