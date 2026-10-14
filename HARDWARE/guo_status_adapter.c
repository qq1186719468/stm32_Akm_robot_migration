#include "kaoya_status_adapter.h"

/*
 * MPU6050初始化在MPU6050.c中明确设置为：陀螺仪±500dps、加速度±2g。
 * 因而原始count可分别按65.5count/(deg/s)、16384count/g换算。
 * 轴映射沿用原data_transition()的ROS坐标约定：X=传感器Y，
 * Y=-传感器X，Z=传感器Z。
 */
#define MPU6050_GYRO_COUNTS_PER_DPS  (65.5f)
#define MPU6050_ACCEL_COUNTS_PER_G   (16384.0f)

static int16_t clamp_i16_float(float value)
{
    if (value >= 32767.0f)
    {
        return 32767;
    }
    if (value <= -32768.0f)
    {
        return (int16_t)-32768;
    }
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static int16_t velocity_to_mmps(float velocity_mps)
{
    /* 编码器速度在Balance_task中已经换算为m/s。 */
    return clamp_i16_float(velocity_mps * 1000.0f);
}

static int16_t gyro_to_centi_dps(int16_t raw_count)
{
    return clamp_i16_float(((float)raw_count * 100.0f) /
                           MPU6050_GYRO_COUNTS_PER_DPS);
}

static int16_t accel_to_centi_g(int16_t raw_count)
{
    return clamp_i16_float(((float)raw_count * 100.0f) /
                           MPU6050_ACCEL_COUNTS_PER_G);
}

uint8_t KaoyaStatus_FromMonitorSnapshot(
    const akm_status_snapshot_t *snapshot,
    kaoya_status_payload_t *payload)
{
    float forward_speed;

    if ((snapshot == 0) || (payload == 0))
    {
        return 0u;
    }

    /* 电池已经在MonitorTask转换为mV，不能再次乘1000。 */
    payload->battery_mv = snapshot->battery_mv;
    payload->temperature_centi = 0;
    forward_speed = (snapshot->motor_a_actual_mps +
                     snapshot->motor_b_actual_mps) * 0.5f;
    payload->vx_mmps = velocity_to_mmps(forward_speed);
    payload->vy_mmps = 0;

    /*
     * 阿克曼车的Move_Z是转向控制量而不是角速度。当前工程没有可靠的
     * IMU融合航向速率/转向角速度快照，因此这里明确填0，避免伪造单位。
     */
    payload->vz_mradps = 0;

    payload->gx_centi_dps = gyro_to_centi_dps(
        snapshot->gyro_corrected_counts[1]);
    payload->gy_centi_dps = gyro_to_centi_dps(
        (int16_t)-snapshot->gyro_corrected_counts[0]);
    payload->gz_centi_dps = gyro_to_centi_dps(
        snapshot->gyro_corrected_counts[2]);

    payload->ax_centi_g = accel_to_centi_g(
        snapshot->accel_corrected_counts[1]);
    payload->ay_centi_g = accel_to_centi_g(
        (int16_t)-snapshot->accel_corrected_counts[0]);
    payload->az_centi_g = accel_to_centi_g(
        snapshot->accel_corrected_counts[2]);

    return 1u;
}
