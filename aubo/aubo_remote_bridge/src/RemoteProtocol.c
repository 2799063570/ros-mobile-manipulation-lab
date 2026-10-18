#include "aubo_remote_bridge/RemoteProtocol.h"
#include <string.h>

uint16_t Remote_GetU16(const uint8_t *Data) { return (uint16_t)(Data[0] | ((uint16_t)Data[1] << 8)); }
uint32_t Remote_GetU32(const uint8_t *Data) { return Remote_GetU16(Data) | ((uint32_t)Remote_GetU16(Data + 2) << 16); }
int16_t Remote_GetI16(const uint8_t *Data)
{
	uint16_t Value = Remote_GetU16(Data);
	return (int16_t)(Value < 0x8000u ? (int32_t)Value : (int32_t)Value - 65536);
}
void Remote_PutU16(uint8_t *Data, uint16_t Value) { Data[0] = (uint8_t)Value; Data[1] = (uint8_t)(Value >> 8); }
void Remote_PutU32(uint8_t *Data, uint32_t Value) { Remote_PutU16(Data, (uint16_t)Value); Remote_PutU16(Data + 2, (uint16_t)(Value >> 16)); }

/** 函数：CRC-16/CCITT-FALSE，多项式0x1021，初值0xFFFF，无反射/异或 */
uint16_t Remote_CRC(const uint8_t *Data, uint16_t Length)
{
	uint16_t CRC = 0xFFFF, Index;
	uint8_t Bit;
	for (Index = 0; Index < Length; Index ++)
	{
		CRC ^= (uint16_t)Data[Index] << 8;
		for (Bit = 0; Bit < 8; Bit ++) CRC = (uint16_t)((CRC & 0x8000) ? (CRC << 1) ^ 0x1021 : CRC << 1);
	}
	return CRC;
}

void Remote_ParserInit(Remote_ParserTypeDef *Parser) { memset(Parser, 0, sizeof(*Parser)); }
static void Remote_Discard(Remote_ParserTypeDef *Parser, uint8_t Count)
{
	Parser->Used = (uint8_t)(Parser->Used - Count);
	memmove(Parser->Buffer, Parser->Buffer + Count, Parser->Used);
}

/** 函数：逐字节解析；非法版本/长度/CRC向后滑动一字节，保留内部可能的帧头 */
uint8_t Remote_ParseByte(Remote_ParserTypeDef *Parser, uint8_t Byte, Remote_FrameTypeDef *Frame)
{
	uint8_t Total;
	if (Parser->Used == REMOTE_MAX_FRAME) Remote_Discard(Parser, 1);
	Parser->Buffer[Parser->Used ++] = Byte;
	while (Parser->Used >= 2)
	{
		if (Parser->Buffer[0] != 0xA5 || Parser->Buffer[1] != 0x5A) { Remote_Discard(Parser, 1); continue; }
		if (Parser->Used < 7) return 0;
		if (Parser->Buffer[2] != REMOTE_VERSION || Parser->Buffer[6] > REMOTE_MAX_PAYLOAD)
		{
			Parser->Errors ++; Remote_Discard(Parser, 1); continue;
		}
		Total = (uint8_t)(Parser->Buffer[6] + 9);
		if (Parser->Used < Total) return 0;
		if (Remote_CRC(Parser->Buffer + 2, (uint16_t)(Total - 4)) != Remote_GetU16(Parser->Buffer + Total - 2))
		{
			Parser->Errors ++; Remote_Discard(Parser, 1); continue;
		}
		Frame->Type = Parser->Buffer[3];
		Frame->Sequence = Remote_GetU16(Parser->Buffer + 4);
		Frame->Length = Parser->Buffer[6];
		memcpy(Frame->Payload, Parser->Buffer + 7, Frame->Length);
		Remote_Discard(Parser, Total);
		return 1;
	}
	return 0;
}

uint8_t Remote_Encode(const Remote_FrameTypeDef *Frame, uint8_t *Data)
{
	uint8_t Total;
	if (Frame->Length > REMOTE_MAX_PAYLOAD) return 0;
	Total = (uint8_t)(Frame->Length + 9);
	Data[0] = 0xA5; Data[1] = 0x5A; Data[2] = REMOTE_VERSION; Data[3] = Frame->Type;
	Remote_PutU16(Data + 4, Frame->Sequence); Data[6] = Frame->Length;
	memcpy(Data + 7, Frame->Payload, Frame->Length);
	Remote_PutU16(Data + Total - 2, Remote_CRC(Data + 2, (uint16_t)(Total - 4)));
	return Total;
}
