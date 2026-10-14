#ifndef __TFT_DIAG_H
#define __TFT_DIAG_H

/* Low-rate TFT diagnostic page; it consumes no framebuffer and no RTOS task. */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AKM_TFT_DIAG_ENABLE
#define AKM_TFT_DIAG_ENABLE (1u)
#endif

void TftDiag_Init(void);
void TftDiag_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __TFT_DIAG_H */
