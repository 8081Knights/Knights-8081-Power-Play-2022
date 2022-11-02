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

        robot.frontRight().setPower(0.8);
        sleep(2000);
        robot.frontRight().setPower(0);


        robot.frontLeft().setPower(0.8);
        sleep(2000);
        robot.frontLeft().setPower(0);


        robot.backRight().setPower(0.8);
        sleep(2000);
        robot.backRight().setPower(0);


        robot.backLeft().setPower(0.8);
        sleep(2000);
        robot.backLeft().setPower(0);
    }
}
