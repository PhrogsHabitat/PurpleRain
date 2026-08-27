package org.firstinspires.ftc.teamcode.Purple.Utils;

import com.qualcomm.robotcore.util.RobotLog;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;

/**
 * Proxy for sending DualSense commands to a Driver Hub companion app.
 * Uses UDP to broadcast trigger states to the network.
 */
public class DualSenseProxy
{
	private static final String TAG = "DualSenseProxy";
	private static final int PROXY_PORT = 5555;
	
	private DatagramSocket socket;
	private InetAddress broadcastAddress;

	public DualSenseProxy ()
	{
		try
		{
			socket = new DatagramSocket();
			socket.setBroadcast(true);
			// Assuming standard FTC subnet
			broadcastAddress = InetAddress.getByName("192.168.43.255");
		} catch (Exception e)
		{
			RobotLog.ee(TAG, "Failed to initialize DualSense Proxy: " + e.getMessage());
		}
	}

	/**
	 * Sends a trigger command to the proxy.
	 * 
	 * @param gamepadIndex 0 or 1
	 * @param isRight      True for right, false for left
	 * @param mode         Trigger mode value
	 * @param params       Parameters
	 */
	public void sendTriggerCommand (int gamepadIndex, boolean isRight, int mode, int... params)
	{
		if (socket == null) return;

		try
		{
			ByteBuffer buffer = ByteBuffer.allocate(16);
			buffer.put((byte) gamepadIndex);
			buffer.put((byte) (isRight ? 1 : 0));
			buffer.put((byte) mode);
			buffer.put((byte) params.length);
			for (int p : params) buffer.put((byte) p);

			byte[] data = buffer.array();
			DatagramPacket packet = new DatagramPacket(data, data.length, broadcastAddress, PROXY_PORT);
			socket.send(packet);
		} catch (Exception e)
		{
			// Ignore network errors to avoid lagging the main loop
		}
	}

	public void close ()
	{
		if (socket != null)
		{
			socket.close();
		}
	}
}
