# state编号见RemoteProtocol.h。command_result是逐请求应答；motion_state是当前动作快照。
uint32 session
uint16 request_id
uint8 command
uint8 state
string message
