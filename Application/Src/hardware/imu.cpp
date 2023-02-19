#include "../../Inc/hardware/imu.h"

namespace hardware
{
    IMU::IMU(float sampling_period)
        : sampling_period(sampling_period),
          theta(0),
          pre_gyro_z(0),
          offset_gz(0),
          offset_ax(0) {}

    uint8_t IMU::read_byte(uint8_t reg)
    {
        uint8_t rx_data[2];
        uint8_t tx_data[2];

        tx_data[0] = reg | 0x80;
        tx_data[1] = 0x00; // dummy

        Write_GPIO(SPI_CS, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
        Write_GPIO(SPI_CS, GPIO_PIN_SET);

        return rx_data[1];
    }

    void IMU::write_byte(uint8_t reg, uint8_t data)
    {
        uint8_t rx_data[2];
        uint8_t tx_data[2];

        tx_data[0] = reg & 0x7F;
        //   tx_data[0] = reg | 0x00;
        tx_data[1] = data; // write data

        Write_GPIO(SPI_CS, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
        Write_GPIO(SPI_CS, GPIO_PIN_SET);
    }

    void IMU::Initialize()
    {
        uint8_t who_am_i;
        Write_GPIO(SPI_CS, GPIO_PIN_SET);
        __HAL_SPI_ENABLE(&hspi1); // clockが動かないように、あらかじめEnableにしておく

        HAL_Delay(100);                          // wait start up
        who_am_i = read_byte(0x75);              // read who am i
        printf("who_am_i = 0x%x\r\n", who_am_i); // check who am i value
        HAL_Delay(10);
        while (who_am_i != 0x70)
        {
            who_am_i = read_byte(0x75);
            printf("who_am_i = 0x%x\r\n", who_am_i);
            HAL_Delay(20);
        }

        HAL_Delay(50);
        write_byte(0x68, 0x00); // set pwr_might (20MHz)
        HAL_Delay(50);
        write_byte(0x1A, 0x00); // set config (FSYNCはNC)
        HAL_Delay(50);
        write_byte(0x1B, 0x18); // set gyro config (2000dps)
        HAL_Delay(50);
        write_byte(0x1C, 0x08); // set acc config (4g)
        HAL_Delay(50);
        // write_byte(0x1D, 0x00); // LPF (Accelerometer, Bandwidth460 Hz)
        // HAL_Delay(50);
    }

    void IMU::CalcOffset()
    {
        int16_t gz_raw;
        float gz_sum = 0;
        Write_GPIO(BACK_LEFT_LED, GPIO_PIN_SET);
        for (int i = 0; i < 500; i++)
        {
            // H:8bit shift, Link h and l
            gz_raw = (int16_t)((int16_t)(read_byte(0x47) << 8) | read_byte(0x48));
            gyro_z = (float)(gz_raw) / gyro_factor * M_PI / 180.0f; // dps to rad/sec

            gz_sum += gyro_z;
            HAL_Delay(1);
        }
        offset_gz = gz_sum / 500.0f;

        int16_t ax_raw;
        float ax_sum = 0;
        Write_GPIO(BACK_RIGHT_LED, GPIO_PIN_SET);
        for (int i = 0; i < 500; i++)
        {
            // H:8bit shift, Link h and l
            ax_raw = (int16_t)((int16_t)(read_byte(0x3D) << 8) | read_byte(0x3E));
            acc_x = (float)(ax_raw) / acc_factor;

            ax_sum += acc_x;
            HAL_Delay(1);
        }
        // turn off led
        Write_GPIO(BACK_LEFT_LED, GPIO_PIN_RESET);
        Write_GPIO(BACK_RIGHT_LED, GPIO_PIN_RESET);

        offset_ax = ax_sum / 500.0f;
    }

    void IMU::Update()
    {
        UpdateGyro();
        UpdateAcc();
    }

    void IMU::UpdateGyro()
    {
        int16_t gz_raw;

        // H:8bit shift, Link h and l
        gz_raw = (int16_t)((int16_t)(read_byte(0x47) << 8) | read_byte(0x48));
        gyro_z = (float)(gz_raw) / gyro_factor * M_PI / 180.0f - offset_gz; // dps to deg/sec
    }

    void IMU::UpdateAcc()
    {
        int16_t ax_raw;

        // H:8bit shift, Link h and l
        ax_raw = (int16_t)((int16_t)(read_byte(0x3D) << 8) | read_byte(0x3E));
        acc_x = (float)(ax_raw) / acc_factor - offset_ax;
    }

    float IMU::GetAngle()
    {
        // Euler method
        // theta += gyro_z * sampling_period;

        // Bilinear transform
        theta += (gyro_z + pre_gyro_z) * sampling_period * 0.5;
        pre_gyro_z = gyro_z;
        return theta;
    }

    float IMU::GetAngularVelocity()
    {
        return gyro_z;
    }

    float IMU::GetAccX()
    {
        return acc_x * g;
    }

    void IMU::ResetTheta()
    {
        theta = 0.0;
    }

} // namespace hardware