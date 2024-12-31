// #ifndef IMU_H
// #define IMU_H

/* The mbed library */
#include <mbed.h>
#include <drivers/LSM6DSL/LSM6DSLSensor.h> // Include the LSM6DSL driver
/* Header file for the task manager library */
#include <utils/task.hpp>

namespace periodics
{
    /**
     * @brief Class CImuLSM6DSL
     * 
     */
    class CImuLSM6DSL : public utils::CTask
    {
    public:
        /* Constructor */
        CImuLSM6DSL(
            uint32_t f_period,
            UnbufferedSerial &f_serial,
            PinName SDA,
            PinName SCL,
            uint8_t address);
        /* Destructor */
        ~CImuLSM6DSL();

        /* Serial callback implementation */
        void ImuPublisherCommand(char const *a, char *b);

    private:
        DevI2C i2c; // I2C instance for communication

        /* I2C initialization routine */
        // virtual void I2C_routine(void);
        /* Run method */
        virtual void _run();

        /** @brief Active flag */
        bool m_isActive;

        /** @brief Serial communication object */
        UnbufferedSerial &m_serial;

        /* Velocity data */
        float m_velocityX;
        float m_velocityY;
        float m_velocityZ;
        int m_velocityStationaryCounter;

        /* LSM6DSL instance */
        LSM6DSLSensor imu;

        /* Variables for storing sensor data */
        float accel[3];
        float gyro[3];
    };

}; // namespace periodics

// #endif // IMU_H
