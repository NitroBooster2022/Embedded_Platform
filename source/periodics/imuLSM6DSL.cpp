#include "imuLSM6DSL.hpp"

namespace periodics
{
    CImuLSM6DSL::CImuLSM6DSL(
        uint32_t f_period,
        UnbufferedSerial &f_serial,
        PinName SDA,
        PinName SCL,
        uint8_t address)
        : utils::CTask(f_period),
          m_isActive(true),
          m_serial(f_serial),
          m_velocityX(0.0),
          m_velocityY(0.0),
          m_velocityZ(0.0),
          m_velocityStationaryCounter(0),
          i2c(SDA, SCL), // Initialize DevI2C
          imu(&i2c, address, NC, NC) // Pass pointer to DevI2C
    {
        printf("Initializing LSM6DSL...\n");

        // Initialize the IMU sensor
        if (imu.init(nullptr) != 0) // Pass nullptr if no specific initialization structure is needed
        {
            printf("Failed to initialize LSM6DSL!\n");
        }
        else
        {
            printf("LSM6DSL initialized successfully.\n");
        }
    }

    CImuLSM6DSL::~CImuLSM6DSL()
    {
        // Nothing specific is needed to clean up the LSM6DSL
    }

    void CImuLSM6DSL::ImuPublisherCommand(char const *a, char *b)
    {
        int l_isActivate = 0;
        uint32_t l_res = sscanf(a, "%d", &l_isActivate);
        if (l_res == 1)
        {
            m_isActive = (l_isActivate >= 1);
            sprintf(b, "ack");
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    void CImuLSM6DSL::_run()
    {
        if (!m_isActive)
            return;

        char buffer[256];
        int32_t accel[3], gyro[3];

        // Read accelerometer data
        imu.get_x_axes(accel);

        // Read gyroscope data
        imu.get_g_axes(gyro);

        // Process accelerometer data to compute velocity
        if (fabs(accel[0]) <= 0.09 && fabs(accel[1]) <= 0.09)
        {
            accel[0] = 0.0;
            accel[1] = 0.0;
            m_velocityX += 0.0 * 0.1; // Δt = f_period * g_baseTick
            m_velocityY += 0.0 * 0.1;
            m_velocityZ += accel[2] * 0.1;
            m_velocityStationaryCounter += 1;
            if (m_velocityStationaryCounter == 15)
            {
                m_velocityX = 0.0;
                m_velocityY = 0.0;
                m_velocityZ = 0.0;
                m_velocityStationaryCounter = 0;
            }
        }
        else
        {
            m_velocityX += accel[0] * 0.1; // Δt = f_period * g_baseTick
            m_velocityY += accel[1] * 0.1;
            m_velocityZ += accel[2] * 0.1;
            m_velocityStationaryCounter = 0;
        }

        // Format the data into a serial buffer
        snprintf(buffer, sizeof(buffer), "@7:%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;;\r\n",
                 gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], m_velocityX, m_velocityY, m_velocityZ);
        m_serial.write(buffer, strlen(buffer));
    }
}; // namespace periodics
