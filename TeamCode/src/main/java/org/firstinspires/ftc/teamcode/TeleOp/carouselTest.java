package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.carouselhardware;


@TeleOp(name="lol doesnt work")
@Disabled
public class carouselTest extends OpMode {
    carouselhardware hw = new carouselhardware();
    @Override

    public void init() {
        hw.init(hardwareMap);
    }

    @Override
    public void loop() {

        hw.frontRight.setPower(0.9);
    }
}
