package org.firstinspires.ftc.teamcode.Purple.Tools;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

@TeleOp(name = "USB Device Scanner", group = "Tools")
public class UsbDeviceScanner extends LinearOpMode
{
	@Override
	public void runOpMode () throws InterruptedException
	{
		UsbManager usbManager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);
		
		telemetry.addLine("Press Start to scan for USB devices on the Robot Controller");
		telemetry.update();

		waitForStart();

		while (opModeIsActive())
		{
			HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
			telemetry.addData("Device Count", deviceList.size());
			
			for (UsbDevice device : deviceList.values())
			{
				telemetry.addLine("---");
				telemetry.addData("Name", device.getDeviceName());
				telemetry.addData("VID", String.format("0x%04X", device.getVendorId()));
				telemetry.addData("PID", String.format("0x%04X", device.getProductId()));
				telemetry.addData("Manufacturer", device.getManufacturerName());
				telemetry.addData("Product", device.getProductName());
			}
			
			telemetry.update();
			sleep(2000);
		}
	}
}
