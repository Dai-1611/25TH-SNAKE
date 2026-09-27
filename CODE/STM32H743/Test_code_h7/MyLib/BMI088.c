//====================================//
// インクルーチE
//====================================//
#include "main.h"
#include "BMI088.h"
//====================================//
// グローバル変数の宣
//====================================//
volatile IMUval BMI088val;
/////////////////////////////////////////////////////////////////////
// モジュール吁EBMI088ReadByteG
// 処琁E��要E    持E��レジスタの値を読み出ぁEジャイロセンサ部)
// 引数         reg: レジスタのアドレス
// 戻り値       読み出した値
////////////////////////////////////////////////////////////////////
static uint8_t BMI088readByte(bool sensorType, uint8_t reg)
{
	uint8_t txData[2]={reg | 0x80, 0x0}, rxData[2] = {0x0};

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxData, sizeof(txData), 1000);

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}

	return rxData[1];
}
/////////////////////////////////////////////////////////////////////
// モジュール吁EBMI088WriteByteG
// 処琁E��要E    持E��レジスタに値を書き込む(ジャイロセンサ部)
// 引数         reg: レジスタのアドレス val: 書き込む値
// 戻り値       なぁE
////////////////////////////////////////////////////////////////////
static void BMI088writeByte(bool sensorType, uint8_t reg, uint8_t val)
{
	uint8_t txData[2] = {reg, val}, rxData[2];

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxData, sizeof(txData), 1000);;

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール吁EBMI088ReadAxisDataG
// 処琁E��要E    持E��レジスタの読み出ぁEジャイロセンサ部)
// 引数         reg:レジスタアドレス
// 戻り値       読み出したチE�Eタ
/////////////////////////////////////////////////////////////////////
static void BMI088readAxisData(bool sensorType, uint8_t reg, uint8_t *rxData, uint8_t rxNum)
{
	uint8_t txData[20] = {0}, rxDatabuff[20];

	txData[0] = reg | 0x80; // 送信用チE�Eタに変換

	if(sensorType == ACCELE)
	{
		CSB1_RESET;
	} else {
		CSB2_RESET;
	}

	HAL_SPI_TransmitReceive(&SPI_Handle_IMU, txData, rxDatabuff, rxNum+1, 1000);
	memcpy(rxData,rxDatabuff+1,rxNum); // レジスタ送信時�E受信チE�Eタを除ぁE��コピ�E

	if(sensorType == ACCELE)
	{
		CSB1_SET;
	} else {
		CSB2_SET;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール吁EinitBMI088
// 処琁E��要E    初期設定パラメータの書き込み
// 引数         なぁE
// 戻り値       なぁE
/////////////////////////////////////////////////////////////////////
bool initBMI088(void)
{
	HAL_Delay(20);
	BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // 加速度センサSPIモードに刁E��替ぁESPIダミ�EリーチE
	BMI088writeByte(ACCELE, REG_ACC_PWR_CTRL, 0x04); // 加速度センサノ�Eマルモードに移衁E
	HAL_Delay(10);
	BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // 加速度センサSPIモードに刁E��替ぁESPIダミ�EリーチE
	BMI088val.Aid = BMI088readByte(ACCELE, REG_ACC_CHIP_ID); // ノ�Eマルモード移行前にチップIDを読む
	BMI088val.Gid = BMI088readByte(GYRO, REG_GYRO_CHIP_ID);
	
	
	if (BMI088val.Gid == 0x0f && BMI088val.Aid == 0x1e)
	{
		// コンフィグ設宁E
		// 加速度
		BMI088writeByte(ACCELE, REG_ACC_PWR_CTRL, 0x04); // 加速度センサノ�Eマルモードに移衁E
		BMI088writeByte(ACCELE, REG_ACC_RANGE, 0x01); // レンジめEgに設宁E
		BMI088writeByte(ACCELE, REG_ACC_CONF, 0xAc);  // ODRめE600Hzに設宁E
		HAL_Delay(10);
		
		// ジャイロ
		BMI088writeByte(GYRO, REG_GYRO_SOFTRESET, 0xB6); // ソフトウェアリセチE��
		HAL_Delay(10);
		BMI088writeByte(GYRO, REG_GYRO_BANDWISTH, 0x02); // ODRめEkHz バンドフィルタ116Hzに設宁E
		BMI088writeByte(GYRO, REG_GYRO_RANGE, 0x00);	// レンジめE000dpsに設宁E

		
		//Data Ready Write
			// ==== Accelerometer: bật INT1 output, active-high, map data-ready ====
BMI088writeByte(ACCELE, REG_ACC_INT1_IO_CTRL, 0x0A); // int1_lvl=1(active high) | int1_output_en=1
BMI088writeByte(ACCELE, REG_ACC_INT_MAP_DATA, 0x04); // map DRDY -> INT1
// Nếu accel DR nối vào chân INT2 thay vì INT1 thì dùng:
// BMI088writeByte(ACCELE, REG_ACC_INT2_IO_CTRL, 0x0A);
// BMI088writeByte(ACCELE, REG_ACC_INT_MAP_DATA, 0x40); // map DRDY -> INT2

// ==== Gyroscope: bật new-data interrupt + map ra INT3/INT4 ====
BMI088writeByte(GYRO, REG_GYRO_INT3_INT4_IO_CONF, 0x01); // INT3: push-pull, active high
BMI088writeByte(GYRO, REG_GYRO_INT_CTRL, 0x80);          // bật "data ready" interrupt (bit7 = data_en)
BMI088writeByte(GYRO, REG_GYRO_INT3_INT4_IO_MAP, 0x01);  // map DRDY -> INT3 (bit0)
// Nếu gyro DR nối vào INT4 thì dùng:
// BMI088writeByte(GYRO, REG_GYRO_INT3_INT4_IO_MAP, 0x80);
		BMI088val.Initialized = 1;
		
		return true;
	}
	else
	{
		return false;
	}
}
/////////////////////////////////////////////////////////////////////
// モジュール吁EBMI088getGyro
// 処琁E��要E    角速度の取征E
// 引数         なぁE
// 戻り値       なぁE
/////////////////////////////////////////////////////////////////////
void BMI088getGyro(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}
	uint8_t rawData[6];
	int16_t gyroVal[3];

	// 角速度の生データを取征E
	BMI088readAxisData(GYRO, REG_RATE_X_LSB, rawData, 6); // x,y,z軸の生データを取征E
	// LSBとMSBを結合
	gyroVal[0] = ((rawData[1] << 8) | rawData[0]); // x軸角速度
	gyroVal[1] = ((rawData[3] << 8) | rawData[2]); // y軸角速度
	gyroVal[2] = ((rawData[5] << 8) | rawData[4]); // z軸角速度

	BMI088val.gyro.x = (float)gyroVal[0] / GYROLSB; // x軸角速度[deg/s]
	BMI088val.gyro.y = (float)gyroVal[1] / GYROLSB; // y軸角速度[deg/s]
	BMI088val.gyro.z = (float)gyroVal[2] / GYROLSB; // z軸角速度[deg/s]
}
/////////////////////////////////////////////////////////////////////
// モジュール吁EBMI088getAccele
// 処琁E��要E    加速度の取得（角度補正に使用�E�E
// 引数         なぁE
// 戻り値       なぁE
/////////////////////////////////////////////////////////////////////
void BMI088getAccele(void)
{
#ifdef USE_ACCELE
	if(!BMI088val.Initialized)
	{
		return;
	}
	uint8_t rawData[8];
	int16_t accelVal[3];

	// 加速度の生データを取征E
	BMI088readAxisData(ACCELE, REG_ACC_X_LSB, rawData, 7);
	// LSBとMSBを結合
	// 最初�EチE�Eタは破棁E��めE
	accelVal[0] = ((rawData[2] << 8) | rawData[1]);
	accelVal[1] = ((rawData[4] << 8) | rawData[3]);
	accelVal[2] = ((rawData[6] << 8) | rawData[5]);

	BMI088val.accele.x = (float)accelVal[0] / ACCELELSB; // x軸加速度[g]
	BMI088val.accele.y = (float)accelVal[1] / ACCELELSB * -1; // y軸加速度[g]
	BMI088val.accele.z = (float)accelVal[2] / ACCELELSB; // z軸加速度[g]
#endif
}
/////////////////////////////////////////////////////////////////////
// モジュール吁EBMI088getTemp
// 処琁E��要E    温度の取征E
// 引数         なぁE
// 戻り値       なぁE
/////////////////////////////////////////////////////////////////////
void BMI088getTemp(void)
{
	if(!BMI088val.Initialized)
	{
		return;
	}
	uint8_t rawData[3];
	uint16_t tempValu;
	int16_t tempVal;

	// 温度の生データを取征E
	BMI088readAxisData(ACCELE, REG_TEMP_MSB, rawData, 3);
	// LSBとMSBを結合
	tempValu = (rawData[1] << 3) | (rawData[2] >> 5);
	if (tempValu > 1023)
	{
		tempVal = ~tempValu + 0x8000;
	}
	else
	{
		tempVal = tempValu;
	}

	BMI088val.temp = ((float)tempVal * 0.125F) + 23.0F;
}
