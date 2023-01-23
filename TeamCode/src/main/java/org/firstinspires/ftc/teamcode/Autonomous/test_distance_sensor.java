package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Distance Sensor Test", group="Sydney")
public class test_distance_sensor extends OpMode {

    private final HardwareSoftware robot = new HardwareSoftware();   // Get the Hardware Setup for this robot

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); // the hardwareMap comes from OpMode class and represents the data from the Android COnfiguration of the robot
    }

    @Override
    public void loop() {

        robot.get_left_ultrasonic().measureRange();
        robot.get_right_ultrasonic().measureRange();

        try {
            super.wait(150);
        } catch (Exception e) {
            // ignore any throw
        }

        short left = robot.get_left_ultrasonic().getLastRange();
        short right = robot.get_right_ultrasonic().getLastRange();

        telemetry.addData("distance", "left (%d), right (%d)", left, right);
        telemetry.update();

        try {
            super.wait(2500);
        } catch (Exception e) {
        }
    }
}
