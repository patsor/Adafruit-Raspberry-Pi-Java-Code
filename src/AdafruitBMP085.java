import java.io.IOException;
import java.util.concurrent.TimeUnit;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

public class AdafruitBMP085 {

	private int mode;
	private I2CDevice device;
	private I2CBus bus;
	// Operating Modes
	private final int BMP085_ULTRALOWPOWER = 0;
	private final int BMP085_STANDARD = 1;
	private final int BMP085_HIGHRES = 2;
	private final int BMP085_ULTRAHIGHRES = 3;

	// # BMP085 Registers
	private final int BMP085_CAL_AC1 = 0xAA; // R Calibration data (16 bits)
	private final int BMP085_CAL_AC2 = 0xAC; // R Calibration data (16 bits)
	private final int BMP085_CAL_AC3 = 0xAE; // R Calibration data (16 bits)
	private final int BMP085_CAL_AC4 = 0xB0; // R Calibration data (16 bits)
	private final int BMP085_CAL_AC5 = 0xB2; // R Calibration data (16 bits)
	private final int BMP085_CAL_AC6 = 0xB4; // R Calibration data (16 bits)
	private final int BMP085_CAL_B1 = 0xB6; // R Calibration data (16 bits)
	private final int BMP085_CAL_B2 = 0xB8; // R Calibration data (16 bits)
	private final int BMP085_CAL_MB = 0xBA; // R Calibration data (16 bits)
	private final int BMP085_CAL_MC = 0xBC; // R Calibration data (16 bits)
	private final int BMP085_CAL_MD = 0xBE; // R Calibration data (16 bits)
	private final int BMP085_CONTROL = 0xF4;
	private final int BMP085_TEMPDATA = 0xF6;
	private final int BMP085_PRESSUREDATA = 0xF6;
	private final int BMP085_READTEMPCMD = 0x2E;
	private final int BMP085_READPRESSURECMD = 0x34;

	// Private Fields
	private int cal_AC1 = 0;
	private int cal_AC2 = 0;
	private int cal_AC3 = 0;
	private int cal_AC4 = 0;
	private int cal_AC5 = 0;
	private int cal_AC6 = 0;
	private int cal_B1 = 0;
	private int cal_B2 = 0;
	private int cal_MB = 0;
	private int cal_MC = 0;
	private int cal_MD = 0;

	public AdafruitBMP085() {
		try {
			bus = I2CFactory.getInstance(I2CBus.BUS_1);
			device = bus.getDevice(0x77);
			readCalibrationData();
		} catch (Exception e) {
			e.printStackTrace();
		}
		int mode = 1;
		if ((mode < 0) | (mode > 3))
			this.mode = BMP085_STANDARD;
		else
			this.mode = mode;
	}

	public AdafruitBMP085(int address) {
		try {
			bus = I2CFactory.getInstance(I2CBus.BUS_1);
			device = bus.getDevice(address);
			readCalibrationData();
		} catch (Exception e) {
			e.printStackTrace();
		}
		int mode = 1;
		if ((mode < 0) | (mode > 3))
			this.mode = BMP085_STANDARD;
		else
			this.mode = mode;
	}

	public AdafruitBMP085(int address, int mode) {
		try {
			bus = I2CFactory.getInstance(I2CBus.BUS_1);
			device = bus.getDevice(address);
			readCalibrationData();
		} catch (Exception e) {
			e.printStackTrace();
		}

		if ((mode < 0) | (mode > 3))
			this.mode = BMP085_STANDARD;
		else
			this.mode = mode;
	}

	private int readS16(int register) {
		// "Reads a signed 16-bit value"
		int hi = 0, lo = 0;
//		byte[] buffer = new byte[256];
		try {
			hi = device.read(register);
			if (hi > 127)
				hi -= 256;
			lo = device.read(register + 1);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return (hi << 8) + lo;
	}

	private int readU16(int register) {
		// "Reads an unsigned 16-bit value"
		int hi = 0, lo = 0;
		try {
			hi = device.read(register);
			lo = device.read(register + 1);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return (hi << 8) + lo;
	}

	private void readCalibrationData() {
		// "Reads the calibration data from the IC"
		cal_AC1 = readS16(BMP085_CAL_AC1); // INT16
		cal_AC2 = readS16(BMP085_CAL_AC2); // INT16
		cal_AC3 = readS16(BMP085_CAL_AC3); // INT16
		cal_AC4 = readU16(BMP085_CAL_AC4); // UINT16
		cal_AC5 = readU16(BMP085_CAL_AC5); // UINT16
		cal_AC6 = readU16(BMP085_CAL_AC6); // UINT16
		cal_B1 = readS16(BMP085_CAL_B1); // INT16
		cal_B2 = readS16(BMP085_CAL_B2); // INT16
		cal_MB = readS16(BMP085_CAL_MB); // INT16
		cal_MC = readS16(BMP085_CAL_MC); // INT16
		cal_MD = readS16(BMP085_CAL_MD); // INT16
	}

	private int readRawTemp() {
		// "Reads the raw (uncompensated) temperature from the sensor"
		try {
			device.write(BMP085_CONTROL, (byte) BMP085_READTEMPCMD);
			TimeUnit.MILLISECONDS.sleep(5); // Wait 5ms
		} catch (Exception e) {
			e.printStackTrace();
		}
		return readU16(BMP085_TEMPDATA);
	}

	private int readRawPressure() {
		// "Reads the raw (uncompensated) pressure level from the sensor"
		int msb = 0, lsb = 0, xlsb = 0;
		try {
			device.write(BMP085_CONTROL,
					(byte) (BMP085_READPRESSURECMD + (mode << 6)));

			if (mode == BMP085_ULTRALOWPOWER)
				TimeUnit.MILLISECONDS.sleep(5);
			else if (mode == BMP085_HIGHRES)
				TimeUnit.MILLISECONDS.sleep(14);
			else if (mode == BMP085_ULTRAHIGHRES)
				TimeUnit.MILLISECONDS.sleep(26);
			else
				TimeUnit.MILLISECONDS.sleep(8);

			msb = device.read(BMP085_PRESSUREDATA);
			lsb = device.read(BMP085_PRESSUREDATA + 1);
			xlsb = device.read(BMP085_PRESSUREDATA + 2);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - mode);
	}

	public double readTemperature() {
		// "Gets the compensated temperature in degrees celcius"

		// Read raw temp before aligning it with the calibration values
		int UT = readRawTemp();
		int X1 = ((UT - cal_AC6) * cal_AC5) >> 15;
		int X2 = (cal_MC << 11) / (X1 + cal_MD);
		int B5 = X1 + X2;
		return (((B5 + 8) >> 4) / 10.0);
	}

	public int readPressure() {
		// "Gets the compensated pressure in pascal"
		int p = 0;
		boolean dsValues = false;

		int UT = readRawTemp();
		int UP = readRawPressure();

		// You can use the datasheet values to test the conversion results
		// dsValues = True
		// dsValues = false;

		if (dsValues) {
			UT = 27898;
			UP = 23843;
			cal_AC6 = 23153;
			cal_AC5 = 32757;
			cal_MB = -32768;
			cal_MC = -8711;
			cal_MD = 2868;
			cal_B1 = 6190;
			cal_B2 = 4;
			cal_AC3 = -14383;
			cal_AC2 = -72;
			cal_AC1 = 408;
			cal_AC4 = 32741;
			mode = BMP085_ULTRALOWPOWER;
		}
		// True Temperature Calculations
		int X1 = ((UT - cal_AC6) * cal_AC5) >> 15;
		int X2 = (cal_MC << 11) / (X1 + cal_MD);
		int B5 = X1 + X2;

		// Pressure Calculations
		int B6 = B5 - 4000;
		X1 = (cal_B2 * (B6 * B6) >> 12) >> 11;
		X2 = (cal_AC2 * B6) >> 11;
		int X3 = X1 + X2;
		int B3 = (((cal_AC1 * 4 + X3) << mode) + 2) / 4;

		X1 = (cal_AC3 * B6) >> 13;
		X2 = (cal_B1 * ((B6 * B6) >> 12)) >> 16;
		X3 = ((X1 + X2) + 2) >> 2;
		int B4 = (cal_AC4 * (X3 + 32768)) >> 15;
		int B7 = (UP - B3) * (50000 >> mode);

		if (B7 < 0x80000000) {
			p = (B7 * 2) / B4;
		} else {
			p = (B7 / B4) * 2;
		}

		X1 = (p >> 8) * (p >> 8);
		X1 = (X1 * 3038) >> 16;
		X2 = (-7357 * p) >> 16;

		p = p + ((X1 + X2 + 3791) >> 4);

		return p;
	}

	public double readAltitude() {
		// "Calculates the altitude in meters"
		return (double) (44330.0 * (1.0 - Math.pow(readPressure() / 101325.0,
				0.1903)));
	}

	public double readAltitude(double seaLevelPressure) {
		// "Calculates the altitude in meters"
		return (double) (44330.0 * (1.0 - Math.pow(readPressure()
				/ seaLevelPressure, 0.1903)));
	}

}
