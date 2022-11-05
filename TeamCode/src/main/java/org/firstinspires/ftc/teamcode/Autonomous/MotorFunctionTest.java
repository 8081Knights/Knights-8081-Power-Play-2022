package org.firstinspires.ftc.teamcode.Autonomous;

import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@Autonomous(name = "Motor Functionality Test")
public class MotorFunctionTest extends LinearOpMode {

    HardwareSoftware robot = new HardwareSoftware();



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        waitForStart();

        robot.leftSlide().setPower(0.1);
        robot.rightSlide().setPower(0.1);

        sleep(10000);
    }
}
