#include "tft_diag.h"

#include "st7735s.h"
#include "akm_monitor.h"
#include "control_safety.h"

#include <stdint.h>

static uint8_t append_text(char *out, uint8_t index, const char *text)
{
    while ((*text != '\0') && (index < 21u))
    {
        out[index++] = *text++;
    }
    return index;
}

static uint8_t append_unsigned(char *out, uint8_t index, uint32_t value)
{
    char digits[10];
    uint8_t count;
    uint8_t digit;

    count = 0u;
    do
    {
        digits[count++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while ((value != 0u) && (count < 10u));
    while ((count > 0u) && (index < 21u))
    {
        digit = (uint8_t)digits[--count];
        out[index++] = (char)digit;
    }
    return index;
}

static uint8_t append_signed(char *out, uint8_t index, int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        if (index < 21u) out[index++] = '-';
        magnitude = (uint32_t)(-(value + 1)) + 1u;
    }
    else
    {
        magnitude = (uint32_t)value;
    }
    return append_unsigned(out, index, magnitude);
}

static uint8_t append_battery(char *out, uint8_t index, uint16_t millivolts)
{
    index = append_unsigned(out, index, millivolts / 1000u);
    if (index < 21u) out[index++] = '.';
    if (index < 21u) out[index++] = (char)('0' + ((millivolts / 100u) % 10u));
    if (index < 21u) out[index++] = (char)('0' + ((millivolts / 10u) % 10u));
    return append_text(out, index, "V");
}

static uint8_t append_line_value(char *out, const char *prefix, int32_t value)
{
    uint8_t index;
    index = append_text(out, 0u, prefix);
    return append_signed(out, index, value);
}

static void draw_line(uint8_t line_number, const char *text)
{
    uint8_t index;
    uint8_t x;

    x = 1u;
    for (index = 0u; (index < 21u) && (text[index] != '\0'); index++)
    {
        ST7735S_DrawChar(x, (uint8_t)(line_number * 8u), text[index],
                         ST7735S_GREEN, ST7735S_BLACK);
        x = (uint8_t)(x + 6u);
    }
}

static void draw_snapshot(const akm_status_snapshot_t *snapshot)
{
    char line[22];
    uint8_t index;
    int32_t vehicle_mmps;
    int32_t target_a_mmps;
    int32_t actual_a_mmps;
    int32_t target_b_mmps;
    int32_t actual_b_mmps;

    /* Clear the whole panel once per second; no 40 kB framebuffer is allocated. */
    ST7735S_Fill(ST7735S_BLACK);
    draw_line(0u, "AKM DIAG ST7735");
    if (snapshot->control_state == CONTROL_STATE_READY) draw_line(1u, "STATE:READY");
    else if (snapshot->control_state == CONTROL_STATE_COMMAND_TIMEOUT) draw_line(1u, "STATE:TIMEOUT");
    else if (snapshot->control_state == CONTROL_STATE_HARD_STOP) draw_line(1u, "STATE:HARDSTOP");
    else if (snapshot->control_state == CONTROL_STATE_SELF_CHECK) draw_line(1u, "STATE:CHECK");
    else draw_line(1u, "STATE:STARTUP");

    index = append_text(line, 0u, "BAT:");
    index = append_battery(line, index, snapshot->battery_mv);
    line[index] = '\0'; draw_line(2u, line);

    vehicle_mmps = (int32_t)(((snapshot->motor_a_actual_mps + snapshot->motor_b_actual_mps) * 500.0f));
    index = append_line_value(line, "VX:", vehicle_mmps);
    index = append_text(line, index, "mm/s"); line[index] = '\0'; draw_line(3u, line);

    target_a_mmps = (int32_t)(snapshot->motor_a_target_mps * 1000.0f);
    actual_a_mmps = (int32_t)(snapshot->motor_a_actual_mps * 1000.0f);
    index = append_line_value(line, "MA T", target_a_mmps);
    index = append_line_value(line, "/A", actual_a_mmps); line[index] = '\0'; draw_line(4u, line);

    target_b_mmps = (int32_t)(snapshot->motor_b_target_mps * 1000.0f);
    actual_b_mmps = (int32_t)(snapshot->motor_b_actual_mps * 1000.0f);
    index = append_line_value(line, "MB T", target_b_mmps);
    index = append_line_value(line, "/A", actual_b_mmps); line[index] = '\0'; draw_line(5u, line);

    index = append_line_value(line, "SERVO:", snapshot->servo_ccr2_pwm);
    index = append_text(line, index, "us"); line[index] = '\0'; draw_line(6u, line);

    index = append_line_value(line, "F:", snapshot->guo_frame_count);
    index = append_line_value(line, " E:", snapshot->guo_error_count); line[index] = '\0'; draw_line(7u, line);
    index = append_line_value(line, "DMA:", snapshot->dma_error_count);
    index = append_line_value(line, " TX:", snapshot->tx_complete_count); line[index] = '\0'; draw_line(8u, line);
    index = append_line_value(line, "HEAP:", (int32_t)snapshot->free_heap_bytes);
    index = append_line_value(line, "/", (int32_t)snapshot->minimum_free_heap_bytes); line[index] = '\0'; draw_line(9u, line);
    index = append_line_value(line, "STACK:", snapshot->minimum_stack_high_water_words);
    index = append_text(line, index, "w"); line[index] = '\0'; draw_line(10u, line);
    index = append_line_value(line, "GZ:", snapshot->gyro_corrected_counts[2]);
    index = append_text(line, index, "cnt"); line[index] = '\0'; draw_line(11u, line);
    index = append_line_value(line, "SEQ:", snapshot->sequence); line[index] = '\0'; draw_line(12u, line);
    draw_line(13u, snapshot->iwdg_enabled ? "IWDG:ON" : "IWDG:OFF");
}

void TftDiag_Init(void)
{
#if (AKM_TFT_DIAG_ENABLE == 1u)
    ST7735S_Init();
#endif
}

void TftDiag_Update(void)
{
#if (AKM_TFT_DIAG_ENABLE == 1u)
    akm_status_snapshot_t snapshot;

    AkmMonitor_GetSnapshot(&snapshot);
    draw_snapshot(&snapshot);
#endif
}
