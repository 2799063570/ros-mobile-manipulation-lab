#ifndef AUBO_REMOTE_JOG_PROTOCOL_H
#define AUBO_REMOTE_JOG_PROTOCOL_H
/* MCU/PC共用；按字段编码，不直接发送C结构体。外层仍为RemoteProtocol v2。 */
#include "RemoteProtocol.h"
#ifdef __cplusplus
extern "C" {
#endif
#define REMOTE_JOG_POLL 0x11
#define REMOTE_JOG_INPUT 0x12
#define REMOTE_JOG_FEEDBACK 0x13
#define REMOTE_JOG_ENABLE 1
#define REMOTE_JOG_STOP 2
#define REMOTE_JOG_HOME 3
#define REMOTE_JOG_CANCEL 4
#define REMOTE_JOG_RESET 5
#define REMOTE_JOG_ADC_VALID 1
#define REMOTE_JOG_CALIBRATED 2
#define REMOTE_JOG_DEADMAN 4
#define REMOTE_JOG_UNKNOWN 0
#define REMOTE_JOG_WAITING 1
#define REMOTE_JOG_DISABLED 2
#define REMOTE_JOG_IDLE 3
#define REMOTE_JOG_MOVING 4
#define REMOTE_JOG_STOPPING 5
#define REMOTE_JOG_HOMING 6
#define REMOTE_JOG_HOME_STOPPING 7
#define REMOTE_JOG_FAULT 8

typedef struct {
  uint32_t session;
  uint16_t request_id;
  uint8_t command, flags, frame;
  uint16_t speed; /* 0.1 mm/s，范围1..1000；零向量时允许0。 */
  int16_t axis[3]; /* -1000..1000；方向向量范数超过1000时PC归一化。 */
} RemoteJog_Input;
typedef struct {
  uint32_t session;
  uint16_t request_id;
  uint8_t command, ack_state, controller_state, home_state, flags, joint_mask;
  int16_t joints[6]; /* mrad */
  uint16_t home_error, fault; /* home_error: mrad，65535=未知 */
} RemoteJog_Feedback;

uint8_t RemoteJog_DecodeInput(const Remote_FrameTypeDef *frame, RemoteJog_Input *input);
uint8_t RemoteJog_EncodeInput(const RemoteJog_Input *input, uint16_t sequence, Remote_FrameTypeDef *frame);
void RemoteJog_EncodeFeedback(const RemoteJog_Feedback *feedback, uint16_t sequence, Remote_FrameTypeDef *frame);
uint8_t RemoteJog_DecodeFeedback(const Remote_FrameTypeDef *frame, RemoteJog_Feedback *feedback);
#ifdef __cplusplus
}
#endif
#endif
