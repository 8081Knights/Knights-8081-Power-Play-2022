package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class carouselhardware {

    HardwareMap hw = null;
    public DcMotorEx frontRight = null;




    public void init(HardwareMap ahw){
        hw = ahw;

        frontRight = hw.get(DcMotorEx.class, "motor1");


        frontRight.setPower(0);




    }


}
