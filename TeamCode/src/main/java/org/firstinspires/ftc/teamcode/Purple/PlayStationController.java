// Completely ChatGPT'd this file lmfao
package org.firstinspires.ftc.teamcode.Purple;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.hardware.usb.UsbRequest;
import android.content.Context;

import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;

import java.nio.ByteBuffer;
import java.util.HashMap;

public class PlayStationController {
    private static final String TAG = "PlayStationController";

    // USB Constants
    private static final int SONY_VID = 0x054C;
    private static final int DS4_PID = 0x09CC;
    private static final int DUAL_SENSE_PID = 0x0CE6;

    // HID Report Constants
    private static final byte DS4_OUTPUT_REPORT_ID = 0x05;
    private static final byte DUAL_SENSE_OUTPUT_REPORT_ID = (byte) 0x31;

    private UsbManager usbManager;
    private UsbDevice usbDevice;
    private UsbDeviceConnection connection;
    private UsbInterface usbInterface;
    private UsbEndpoint outputEndpoint;
    private boolean isDualSense = false;

    public PlayStationController(UsbManager usbManager) {
        this.usbManager = usbManager;
        initializeController();
    }

    private void initializeController() {
        // Find PlayStation controller in USB device list
        HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
        for (UsbDevice device : deviceList.values()) {
            if (device.getVendorId() == SONY_VID &&
                    (device.getProductId() == DS4_PID || device.getProductId() == DUAL_SENSE_PID)) {
                this.usbDevice = device;
                this.isDualSense = (device.getProductId() == DUAL_SENSE_PID);
                break;
            }
        }

        if (usbDevice == null) {
            return;
        }

        // Open USB connection
        connection = usbManager.openDevice(usbDevice);
        if (connection == null) {
            return;
        }

        // Claim interface (usually interface 3 for PlayStation controllers)
        for (int i = 0; i < usbDevice.getInterfaceCount(); i++) {
            UsbInterface intf = usbDevice.getInterface(i);
            if (connection.claimInterface(intf, true)) {
                this.usbInterface = intf;
                break;
            }
        }

        // Find output endpoint
        for (int i = 0; i < usbInterface.getEndpointCount(); i++) {
            UsbEndpoint endpoint = usbInterface.getEndpoint(i);
            if (endpoint.getDirection() == UsbConstants.USB_DIR_OUT) {
                this.outputEndpoint = endpoint;
                break;
            }
        }
    }

    /**
     * Send raw binary rumble command to controller
     * @param leftIntensity 0-255 for left/low-frequency motor
     * @param rightIntensity 0-255 for right/high-frequency motor
     */
    public void sendRumbleCommand(int leftIntensity, int rightIntensity) {
        if (connection == null || outputEndpoint == null) return;

        byte[] report = isDualSense ?
                createDualSenseRumbleReport(leftIntensity, rightIntensity) :
                createDS4RumbleReport(leftIntensity, rightIntensity);

        // Send via bulk transfer
        int result = connection.bulkTransfer(outputEndpoint, report, report.length, 0);
    }

    /**
     * DualShock 4 Rumble Report Structure (79 bytes)
     */
    private byte[] createDS4RumbleReport(int left, int right) {
        byte[] report = new byte[79];

        // Report header
        report[0] = DS4_OUTPUT_REPORT_ID;
        report[1] = 0x01; // Unknown, always 1

        // Rumble data (bytes 4-5)
        report[4] = (byte) (right & 0xFF);  // Right motor (high frequency)
        report[5] = (byte) (left & 0xFF);   // Left motor (low frequency)

        // LED control (bytes 6-9)
        report[6] = (byte) 0xFF; // Red
        report[7] = (byte) 0x00; // Green
        report[8] = (byte) 0x00; // Blue
        report[9] = (byte) 0x00; // Flash/brightness

        // CRC32 would go here in real implementation
        return report;
    }

    /**
     * DualSense Rumble Report Structure (48 bytes)
     */
    private byte[] createDualSenseRumbleReport(int left, int right) {
        byte[] report = new byte[48];

        // Report header
        report[0] = DUAL_SENSE_OUTPUT_REPORT_ID;
        report[1] = 0x02; // Unknown, always 2

        // Rumble data (bytes 2-5)
        report[2] = (byte) (right & 0xFF);  // Right motor
        report[3] = (byte) (left & 0xFF);   // Left motor

        // Adaptive trigger support would go here
        // This is significantly more complex and requires detailed protocol knowledge

        return report;
    }

    /**
     * Send LED color command
     */
    public void sendLEDCommand(int red, int green, int blue) {
        if (connection == null || outputEndpoint == null) return;

        byte[] report = isDualSense ?
                createDualSenseLEDReport(red, green, blue) :
                createDS4LEDReport(red, green, blue);

        connection.bulkTransfer(outputEndpoint, report, report.length, 0);
    }

    private byte[] createDS4LEDReport(int r, int g, int b) {
        byte[] report = new byte[79];
        report[0] = DS4_OUTPUT_REPORT_ID;
        report[1] = 0x01;

        // LED position in DS4 report
        report[6] = (byte) (r & 0xFF);
        report[7] = (byte) (g & 0xFF);
        report[8] = (byte) (b & 0xFF);

        return report;
    }

    private byte[] createDualSenseLEDReport(int r, int g, int b) {
        byte[] report = new byte[48];
        report[0] = DUAL_SENSE_OUTPUT_REPORT_ID;
        report[1] = 0x02;

        // DualSense LED control is more complex - simplified version
        // Real implementation would need proper position mapping
        report[44] = (byte) (r & 0xFF);
        report[45] = (byte) (g & 0xFF);
        report[46] = (byte) (b & 0xFF);

        return report;
    }

    /**
     * Send adaptive trigger effect (DualSense only)
     * @param trigger 0=left, 1=right
     * @param mode 0=off, 1=resistance, 2=weapon, 3=vibration
     * @param startPosition 0-255 trigger start position
     * @param effectStrength 0-255 effect intensity
     */
    public void sendTriggerEffect(int trigger, int mode, int startPosition, int effectStrength) {
        if (!isDualSense || connection == null) return;

        byte[] report = createTriggerEffectReport(trigger, mode, startPosition, effectStrength);
        connection.bulkTransfer(outputEndpoint, report, report.length, 0);
    }

    private byte[] createTriggerEffectReport(int trigger, int mode, int start, int strength) {
        byte[] report = new byte[48];
        report[0] = DUAL_SENSE_OUTPUT_REPORT_ID;
        report[1] = 0x02;

        // Trigger effect positions in DualSense report
        int triggerOffset = trigger == 0 ? 0x02 : 0x08;

        report[triggerOffset] = (byte) (mode & 0xFF);
        report[triggerOffset + 1] = (byte) (start & 0xFF);
        report[triggerOffset + 2] = (byte) (strength & 0xFF);

        return report;
    }

    public void close() {
        if (connection != null) {
            if (usbInterface != null) {
                connection.releaseInterface(usbInterface);
            }
            connection.close();
        }
    }
}