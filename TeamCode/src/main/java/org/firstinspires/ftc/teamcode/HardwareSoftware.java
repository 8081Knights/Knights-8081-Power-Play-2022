package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareSoftware {

    HardwareMap hw = null;
    DcMotorEx frontRight = null;
    DcMotorEx backRight  = null;
    DcMotorEx backLeft   = null;
    DcMotorEx frontLeft  = null;
    DcMotorEx turnEncoder = null;



    public void init(HardwareMap ahw){
        hw = ahw;

        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backRight = hw.get(DcMotorEx.class, "backRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");
        turnEncoder = hw.get(DcMotorEx.class, "turnEncoder");

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turnEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        turnEncoder.setPower(0);



    }

    public DcMotorEx frontRight(){
        return frontRight;
    }

    public DcMotorEx backRight(){
        return frontRight;
    }

    public DcMotorEx frontLeft(){
        return frontRight;
    }

    public DcMotorEx backLeft(){
        return frontRight;
    }

    public DcMotorEx turnEncoder() { return turnEncoder;}

}
