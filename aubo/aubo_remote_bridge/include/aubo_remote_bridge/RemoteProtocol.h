#ifndef __REMOTE_PROTOCOL_H
#define __REMOTE_PROTOCOL_H
#include <stdint.h>
#define REMOTE_VERSION 2
#define REMOTE_MAX_PAYLOAD 32
#define REMOTE_MAX_FRAME (REMOTE_MAX_PAYLOAD + 9)
#define REMOTE_POLL 1
#define REMOTE_INPUT 2
#define REMOTE_FEEDBACK 3
#define REMOTE_SOURCE_NONE 0
#define REMOTE_SOURCE_ROS 1
#define REMOTE_SOURCE_DEMO 2
/*命令编号和状态编号同时用于ROS消息与OLED反馈。*/
#define REMOTE_HOME 1
#define REMOTE_ZERO 2
#define REMOTE_CANCEL 3
#define REMOTE_IDLE 0
#define REMOTE_ACCEPTED 1
#define REMOTE_PLANNING 2
#define REMOTE_EXECUTING 3
#define REMOTE_SUCCEEDED 4
#define REMOTE_FAILED 5
#define REMOTE_CANCELING 6
#define REMOTE_CANCELED 7
#define REMOTE_REJECTED 8
#define REMOTE_VERIFYING 9
#define REMOTE_UNKNOWN 10
typedef struct
{
	uint8_t Type, Length;
	uint16_t Sequence;
	uint8_t Payload[REMOTE_MAX_PAYLOAD];
} Remote_FrameTypeDef;
typedef struct
{
	uint8_t Buffer[REMOTE_MAX_FRAME], Used;
	uint32_t Errors;
} Remote_ParserTypeDef;
uint16_t Remote_CRC(const uint8_t *Data, uint16_t Length);
uint16_t Remote_GetU16(const uint8_t *Data);
uint32_t Remote_GetU32(const uint8_t *Data);
int16_t Remote_GetI16(const uint8_t *Data);
void Remote_PutU16(uint8_t *Data, uint16_t Value);
void Remote_PutU32(uint8_t *Data, uint32_t Value);
void Remote_ParserInit(Remote_ParserTypeDef *Parser);
uint8_t Remote_ParseByte(Remote_ParserTypeDef *Parser, uint8_t Byte, Remote_FrameTypeDef *Frame);
uint8_t Remote_Encode(const Remote_FrameTypeDef *Frame, uint8_t *Data);
#endif
