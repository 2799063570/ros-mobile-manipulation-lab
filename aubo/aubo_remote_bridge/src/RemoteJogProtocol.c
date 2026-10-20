#include "aubo_remote_bridge/RemoteJogProtocol.h"
#include <string.h>

uint8_t RemoteJog_DecodeInput(const Remote_FrameTypeDef *frame, RemoteJog_Input *out) {
  RemoteJog_Input input;
  unsigned i;
  if (frame->Type != REMOTE_JOG_INPUT || frame->Length != 18 || frame->Payload[9]) return 0;
  memset(&input, 0, sizeof(input));
  input.session = Remote_GetU32(frame->Payload);
  input.request_id = Remote_GetU16(frame->Payload + 4);
  input.command = frame->Payload[6]; input.flags = frame->Payload[7]; input.frame = frame->Payload[8];
  input.speed = Remote_GetU16(frame->Payload + 10);
  if (input.command > REMOTE_JOG_RESET || input.flags > 7 || input.frame > 1 || input.speed > 1000 ||
      ((input.command == 0) != (input.request_id == 0))) return 0;
  for (i = 0; i < 3; ++i) {
    input.axis[i] = Remote_GetI16(frame->Payload + 12 + 2 * i);
    if (input.axis[i] < -1000 || input.axis[i] > 1000) return 0;
  }
  if (!input.speed && (input.axis[0] || input.axis[1] || input.axis[2])) return 0;
  *out = input; return 1;
}
uint8_t RemoteJog_EncodeInput(const RemoteJog_Input *input, uint16_t sequence, Remote_FrameTypeDef *frame) {
  unsigned i; RemoteJog_Input check;
  memset(frame, 0, sizeof(*frame));
  frame->Type = REMOTE_JOG_INPUT; frame->Sequence = sequence; frame->Length = 18;
  Remote_PutU32(frame->Payload, input->session); Remote_PutU16(frame->Payload + 4, input->request_id);
  frame->Payload[6] = input->command; frame->Payload[7] = input->flags; frame->Payload[8] = input->frame;
  Remote_PutU16(frame->Payload + 10, input->speed);
  for (i = 0; i < 3; ++i) Remote_PutU16(frame->Payload + 12 + 2 * i, (uint16_t)input->axis[i]);
  return RemoteJog_DecodeInput(frame, &check);
}
void RemoteJog_EncodeFeedback(const RemoteJog_Feedback *input, uint16_t sequence, Remote_FrameTypeDef *frame) {
  unsigned i;
  memset(frame, 0, sizeof(*frame));
  frame->Type = REMOTE_JOG_FEEDBACK; frame->Sequence = sequence; frame->Length = 28;
  Remote_PutU32(frame->Payload, input->session); Remote_PutU16(frame->Payload + 4, input->request_id);
  frame->Payload[6] = input->command; frame->Payload[7] = input->ack_state;
  frame->Payload[8] = input->controller_state; frame->Payload[9] = input->home_state;
  frame->Payload[10] = input->flags; frame->Payload[11] = input->joint_mask;
  for (i = 0; i < 6; ++i) Remote_PutU16(frame->Payload + 12 + 2 * i, (uint16_t)input->joints[i]);
  Remote_PutU16(frame->Payload + 24, input->home_error); Remote_PutU16(frame->Payload + 26, input->fault);
}
uint8_t RemoteJog_DecodeFeedback(const Remote_FrameTypeDef *frame, RemoteJog_Feedback *out) {
  unsigned i;
  if (frame->Type != REMOTE_JOG_FEEDBACK || frame->Length != 28) return 0;
  memset(out, 0, sizeof(*out));
  out->session = Remote_GetU32(frame->Payload); out->request_id = Remote_GetU16(frame->Payload + 4);
  out->command = frame->Payload[6]; out->ack_state = frame->Payload[7];
  out->controller_state = frame->Payload[8]; out->home_state = frame->Payload[9];
  out->flags = frame->Payload[10]; out->joint_mask = frame->Payload[11];
  for (i = 0; i < 6; ++i) out->joints[i] = Remote_GetI16(frame->Payload + 12 + 2 * i);
  out->home_error = Remote_GetU16(frame->Payload + 24); out->fault = Remote_GetU16(frame->Payload + 26);
  return 1;
}
