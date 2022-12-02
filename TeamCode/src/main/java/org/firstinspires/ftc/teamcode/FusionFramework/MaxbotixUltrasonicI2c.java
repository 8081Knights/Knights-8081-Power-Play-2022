/*
Copyright (c) 2019 David M. Wheeler

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode.FusionFramework;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


//import android.support.annotation.Nullable;

/**
 * {@link I2cDeviceSynchImpl} is a utility class that makes it easy to read or write data to
 * an instance of {@link I2cDevice}. Its functionality is exposed through the {@link I2cDeviceSynch}
 * interface. Please see that interface, and the {@link I2cDeviceSynchImpl} constructor here, for
 * further information.
 *
 * @see I2cDeviceSynchImpl#I2cDeviceSynchImpl(I2cDevice, I2cAddr, boolean)
 * @see I2cDeviceSynch
 * @see I2cDevice
 */
@SuppressWarnings("WeakerAccess")
@I2cDeviceType
@DeviceProperties(name = "MB-12xx Ultrasonic Distance Sensor", description = "Distance Sensor from Maxbotix", xmlTag = "MB12XX", builtIn = false)
public final class MaxbotixUltrasonicI2c extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    public static I2cAddr MAXBOTIX_DEFAULT_I2C_ADDRESS = I2cAddr.create8bit(0xE0);

    public static int START_RANGE_READING = 0x51;
    public static int GET_LAST_RANGE      = 0x01;   // This is just a read of the address (last bit of addr is 1)
    public static int CHANGE_ADDR_CMD1    = 0xAA;
    public static int CHANGE_ADDR_CMD2    = 0xA5;



    @Override
    public Manufacturer getManufacturer()
    {
        // Maxbotics is not a known manufacturer to FTC
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "Maxbotix MB12xx I2C Ultrasonc Distance Sensor";
    }

    public MaxbotixUltrasonicI2c(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(MAXBOTIX_DEFAULT_I2C_ADDRESS);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void writeByte(final int address, byte value)
    {
        byte[] data = new byte[1];
        data[0] = value;
        deviceClient.write(address, data);
    }

    protected short readShort(int address)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(address, 2));
    }

    // The commands that the Maxbotix Responds to

    public void measureRange() {
        writeByte( 0x00, (byte)START_RANGE_READING);
    }

    public short getLastRange() {
        return readShort( GET_LAST_RANGE );
    }
}