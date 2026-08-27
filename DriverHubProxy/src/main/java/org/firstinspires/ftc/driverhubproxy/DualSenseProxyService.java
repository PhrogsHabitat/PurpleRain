package org.firstinspires.ftc.driverhubproxy;

import android.app.Service;
import android.content.Intent;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.IBinder;
import android.util.Log;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.util.zip.CRC32;

public class DualSenseProxyService extends Service
{
	private static final String TAG = "DualSenseProxy";
	private static final int PROXY_PORT = 5555;
	
	private DatagramSocket socket;
	private boolean running = false;
	
	private UsbManager usbManager;
	private UsbDeviceConnection connection;
	private UsbEndpoint outputEndpoint;
	private final byte[] dualSenseReport = new byte[64];
	private final CRC32 crc32 = new CRC32();

	@Override
	public void onCreate ()
	{
		super.onCreate();
		usbManager = (UsbManager) getSystemService(USB_SERVICE);
		startUdpServer();
	}

	private void startUdpServer ()
	{
		running = true;
		new Thread(() -> {
			try
			{
				socket = new DatagramSocket(PROXY_PORT);
				byte[] buffer = new byte[64];
				while (running)
				{
					DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
					socket.receive(packet);
					handlePacket(packet.getData(), packet.getLength());
				}
			} catch (Exception e)
			{
				Log.e(TAG, "UDP Server Error: " + e.getMessage());
			}
		}).start();
	}

	private void handlePacket (byte[] data, int length)
	{
		if (length < 4) return;
		ByteBuffer bb = ByteBuffer.wrap(data, 0, length);
		int gamepadIndex = bb.get() & 0xFF;
		boolean isRight = bb.get() != 0;
		int mode = bb.get() & 0xFF;
		int paramCount = bb.get() & 0xFF;
		int[] params = new int[paramCount];
		for (int i = 0; i < paramCount; i++) params[i] = bb.get() & 0xFF;

		applyToController(isRight, mode, params);
	}

	private void applyToController (boolean isRight, int mode, int[] params)
	{
		if (connection == null || outputEndpoint == null)
		{
			findAndOpenController();
			if (connection == null) return;
		}

		// Re-initialize report if needed
		if (dualSenseReport[0] == 0)
		{
			dualSenseReport[0] = 0x31;
			dualSenseReport[1] = 0x02;
			dualSenseReport[2] = (byte) 0xFF;
		}

		int offset = isRight ? 22 : 11;
		dualSenseReport[offset] = (byte) mode;
		for (int i = 0; i < 10; i++)
		{
			dualSenseReport[offset + 1 + i] = (i < params.length) ? (byte) params[i] : 0;
		}

		// CRC32
		crc32.reset();
		crc32.update(dualSenseReport, 0, dualSenseReport.length - 4);
		long crc = crc32.getValue();
		int len = dualSenseReport.length;
		dualSenseReport[len - 4] = (byte) (crc & 0xFF);
		dualSenseReport[len - 3] = (byte) ((crc >> 8) & 0xFF);
		dualSenseReport[len - 2] = (byte) ((crc >> 16) & 0xFF);
		dualSenseReport[len - 1] = (byte) ((crc >> 24) & 0xFF);

		connection.bulkTransfer(outputEndpoint, dualSenseReport, dualSenseReport.length, 0);
	}

	private void findAndOpenController ()
	{
		for (UsbDevice device : usbManager.getDeviceList().values())
		{
			if (device.getVendorId() == 0x054C && (device.getProductId() == 0x0CE6 || device.getProductId() == 0x0DF2))
			{
				connection = usbManager.openDevice(device);
				if (connection == null) continue;

				UsbInterface intf = device.getInterface(3); // Usually 3 for DualSense
				connection.claimInterface(intf, true);

				for (int i = 0; i < intf.getEndpointCount(); i++)
				{
					UsbEndpoint ep = intf.getEndpoint(i);
					if (ep.getDirection() == UsbConstants.USB_DIR_OUT)
					{
						outputEndpoint = ep;
						break;
					}
				}
				break;
			}
		}
	}

	@Override
	public void onDestroy ()
	{
		running = false;
		if (socket != null) socket.close();
		if (connection != null) connection.close();
		super.onDestroy();
	}

	@Override
	public IBinder onBind (Intent intent) { return null; }
}
