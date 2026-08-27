package org.firstinspires.ftc.driverhubproxy;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity
{
	private static final String ACTION_USB_PERMISSION = "org.firstinspires.ftc.driverhubproxy.USB_PERMISSION";
	private UsbManager usbManager;
	private TextView statusText;

	@Override
	protected void onCreate (Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(android.R.layout.simple_list_item_1);
		statusText = (TextView) findViewById(android.R.id.text1);
		statusText.setText("DualSense Proxy Running\nEnsure controller is plugged in.");

		usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

		Intent intent = new Intent(this, DualSenseProxyService.class);
		startService(intent);

		requestUsbPermission();
	}

	private void requestUsbPermission ()
	{
		for (UsbDevice device : usbManager.getDeviceList().values())
		{
			if (device.getVendorId() == 0x054C)
			{
				PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), PendingIntent.FLAG_IMMUTABLE);
				IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
				registerReceiver(usbReceiver, filter);
				usbManager.requestPermission(device, permissionIntent);
				break;
			}
		}
	}

	private final BroadcastReceiver usbReceiver = new BroadcastReceiver()
	{
		public void onReceive (Context context, Intent intent)
		{
			String action = intent.getAction();
			if (ACTION_USB_PERMISSION.equals(action))
			{
				synchronized (this)
				{
					UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
					{
						if (device != null)
						{
							Toast.makeText(context, "USB Permission Granted", Toast.LENGTH_SHORT).show();
							// Restart service to pick up the device
							stopService(new Intent(context, DualSenseProxyService.class));
							startService(new Intent(context, DualSenseProxyService.class));
						}
					}
				}
			}
		}
	};

	@Override
	protected void onDestroy ()
	{
		super.onDestroy();
		try {
			unregisterReceiver(usbReceiver);
		} catch (Exception e) {}
	}
}
