# 最近64条session/request_id去重；编号不要复用，重连后旧请求不自动执行。
uint32 session
uint16 request_id
uint8 command
