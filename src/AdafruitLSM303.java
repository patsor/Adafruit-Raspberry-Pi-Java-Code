import java.util.ArrayList;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

public class AdafruitLSM303 {

	private I2CDevice accDevice, magDevice;
	// Minimal constants carried over from Arduino library
	private final int LSM303_ADDRESS_ACCEL = (0x32 >> 1); // 0011001x
	private final int LSM303_ADDRESS_MAG = (0x3C >> 1); // 0011110x
	// Default Type
	private final int LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20; // 00000111 rw
	private final int LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23; // 00000000 rw
	private final int LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28;
	private final int LSM303_REGISTER_ACCEL_OUT_X_H_A = 0x29;
	private final int LSM303_REGISTER_ACCEL_OUT_Y_L_A = 0x2a;
	private final int LSM303_REGISTER_ACCEL_OUT_Y_H_A = 0x2b;
	private final int LSM303_REGISTER_ACCEL_OUT_Z_L_A = 0x2c;
	private final int LSM303_REGISTER_ACCEL_OUT_Z_H_A = 0x2d;
	private final int LSM303_REGISTER_MAG_CRB_REG_M = 0x01;
	private final int LSM303_REGISTER_MAG_MR_REG_M = 0x02;
	private final int LSM303_REGISTER_MAG_OUT_X_H_M = 0x03;
	private final int LSM303_REGISTER_MAG_OUT_X_L_M = 0x04;
	private final int LSM303_REGISTER_MAG_OUT_Z_H_M = 0x05;
	private final int LSM303_REGISTER_MAG_OUT_Z_L_M = 0x06;
	private final int LSM303_REGISTER_MAG_OUT_Y_H_M = 0x07;
	private final int LSM303_REGISTER_MAG_OUT_Y_L_M = 0x08;

	// Gain settings for setMagGain()
	private final int LSM303_MAGGAIN_1_3 = 0x20; // +/- 1.3
	private final int LSM303_MAGGAIN_1_9 = 0x40; // +/- 1.9
	private final int LSM303_MAGGAIN_2_5 = 0x60; // +/- 2.5
	private final int LSM303_MAGGAIN_4_0 = 0x80; // +/- 4.0
	private final int LSM303_MAGGAIN_4_7 = 0xA0; // +/- 4.7
	private final int LSM303_MAGGAIN_5_6 = 0xC0; // +/- 5.6
	private final int LSM303_MAGGAIN_8_1 = 0xE0; // +/- 8.1

	public AdafruitLSM303(boolean hires) {
		try {
			I2CBus bus = I2CFactory.getInstance(I2CBus.BUS_1);

			// Accelerometer and magnetometer are at different I2C
			// addresses, so invoke a separate I2C instance for each
			accDevice = bus.getDevice(LSM303_ADDRESS_ACCEL);
			magDevice = bus.getDevice(LSM303_ADDRESS_MAG);
			// Enable the accelerometer
			accDevice.write(LSM303_REGISTER_ACCEL_CTRL_REG1_A, (byte) 0x27);
			// Select hi-res (12-bit) or low-res (10-bit) output mode.
			// Low-res mode uses less power and sustains a higher update rate,
			// output is padded to compatible 12-bit units.
			if (hires)
				accDevice.write(LSM303_REGISTER_ACCEL_CTRL_REG4_A,
						(byte) 0b00001000);
			else
				accDevice.write(LSM303_REGISTER_ACCEL_CTRL_REG4_A, (byte) 0);

			// Enable the magnetometer
			magDevice.write(LSM303_REGISTER_MAG_MR_REG_M, (byte) 0x00);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	// Interpret signed 12-bit acceleration component from list
	private int accel12(int lowres, int hires) {
		int n = lowres | (hires << 8); // Low, high bytes
		if (n > 32767)
			n -= 65536; // 2's complement signed
		return n >> 4; // 12-bit resolution
	}

	// Interpret signed 16-bit magnetometer component from list
	private int mag16(int lowres, int hires) {
		int n = (hires << 8) | lowres; // High, low bytes
		// return n if n < 32768 else n - 65536; // 2's complement signed
		if (n < 32768)
			return n;
		else
			return n - 65536;
	}

	public ArrayList<Integer> read() {
		// Read the accelerometer

		ArrayList<Integer> res = new ArrayList<Integer>();
		// int ax, axHR, ay, ayHR, az, azHR, mx, mxHR, my, myHR, mz, mzHR;
		// axLR = accDevice.read(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
		// axHR = accDevice.read(LSM303_REGISTER_ACCEL_OUT_X_H_A | 0x80);
		// ayLR = accDevice.read(LSM303_REGISTER_ACCEL_OUT_Y_L_A | 0x80);
		// ayHR = accDevice.read(LSM303_REGISTER_ACCEL_OUT_Y_H_A | 0x80);
		// azLR = accDevice.read(LSM303_REGISTER_ACCEL_OUT_Z_L_A | 0x80);
		// azHR = accDevice.read(LSM303_REGISTER_ACCEL_OUT_Z_H_A | 0x80);
		//
		// mxHR = magDevice.read(LSM303_REGISTER_MAG_OUT_X_H_M);
		// mxLR = magDevice.read(LSM303_REGISTER_MAG_OUT_X_L_M);
		// myHR = magDevice.read(LSM303_REGISTER_MAG_OUT_Y_H_M);
		// myLR = magDevice.read(LSM303_REGISTER_MAG_OUT_Y_L_M);
		// mzHR = magDevice.read(LSM303_REGISTER_MAG_OUT_Z_H_M);
		// mzLR = magDevice.read(LSM303_REGISTER_MAG_OUT_Z_L_M);

		// Read the magnetometer
		try {
			res.add(accel12(
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80),
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_X_H_A | 0x80)));
			res.add(accel12(
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Y_L_A | 0x80),
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Y_H_A | 0x80)));
			res.add(accel12(
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Z_L_A | 0x80),
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Z_H_A | 0x80)));
			res.add(mag16(magDevice.read(LSM303_REGISTER_MAG_OUT_X_L_M),
					magDevice.read(LSM303_REGISTER_MAG_OUT_X_H_M)));
			res.add(mag16(magDevice.read(LSM303_REGISTER_MAG_OUT_Y_L_M),
					magDevice.read(LSM303_REGISTER_MAG_OUT_Y_H_M)));
			res.add(mag16(magDevice.read(LSM303_REGISTER_MAG_OUT_Z_L_M),
					magDevice.read(LSM303_REGISTER_MAG_OUT_Z_H_M)));
		} catch (Exception e) {
			e.printStackTrace();
		}
		return res;
	}
	
	public int getAx(){
		try{
			return accel12(
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80),
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_X_H_A | 0x80));
		}
		catch(Exception e){ e.printStackTrace();}
		return 0;
	}
	
	public int getAy(){
		try{
			return accel12(
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Y_L_A | 0x80),
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Y_H_A | 0x80));
		}
		catch(Exception e){ e.printStackTrace();}
		return 0;
	}
	
	public int getAz(){
		try{
			return accel12(
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Z_L_A | 0x80),
					accDevice.read(LSM303_REGISTER_ACCEL_OUT_Z_H_A | 0x80));
		}
		catch(Exception e){ e.printStackTrace();}
		return 0;
	}
	
	public int getMx(){
		try{
			return mag16(magDevice.read(LSM303_REGISTER_MAG_OUT_X_L_M),
					magDevice.read(LSM303_REGISTER_MAG_OUT_X_H_M));
		}
		catch(Exception e){ e.printStackTrace();}
		return 0;
	}
	
	public int getMy(){
		try{
			return mag16(magDevice.read(LSM303_REGISTER_MAG_OUT_Y_L_M),
					magDevice.read(LSM303_REGISTER_MAG_OUT_Y_H_M));
		}
		catch(Exception e){ e.printStackTrace();}
		return 0;
	}
	
	public int getMz(){
		try{
			return mag16(magDevice.read(LSM303_REGISTER_MAG_OUT_Z_L_M),
					magDevice.read(LSM303_REGISTER_MAG_OUT_Z_H_M));
		}
		catch(Exception e){ e.printStackTrace();}
		return 0;
	}

	public void setMagGain(){
		try{
		magDevice.write(LSM303_REGISTER_MAG_CRB_REG_M,
				(byte) LSM303_MAGGAIN_1_3);
		} catch(Exception e){e.printStackTrace();}
	}

	public void setMagGain(int gain){
		try{
		magDevice.write(LSM303_REGISTER_MAG_CRB_REG_M, (byte) gain);
		} catch(Exception e){e.printStackTrace();}
	}
}
