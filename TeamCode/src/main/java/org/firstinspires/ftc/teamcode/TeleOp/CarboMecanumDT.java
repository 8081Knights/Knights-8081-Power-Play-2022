package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarboMecanumDT extends OpMode {
    HardwareMap hw = null;
    DcMotorEx frontRight = null;
    DcMotorEx backRight  = null;
    DcMotorEx backLeft   = null;
    DcMotorEx frontLeft  = null;
    double speed = 0;




    @Override
    public void init() {

        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backRight = hw.get(DcMotorEx.class, "backRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        speed = gamepad1.left_stick_y;
        frontRight.setPower(speed);
        frontLeft.setPower(Math.abs(speed) *(-1));


    }
}
