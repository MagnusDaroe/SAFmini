sPROGRAM PLC_PRG
VAR
	state: uint := 0;
	RFID: dc_ecp.Dtm425;
	RfidData: dc_ecp.st_CPF_RfidData;
	correctedID: UINT;
	TCPclient: dc_ecp.TcpClientByteStream;
	datetimeFB: DTU.GetDateAndTime;
	datetime: DATE_AND_TIME;
	CarrierID: STRING;
	datetimestring: STRING;
	XML: STRING(150);
	PLCID: STRING := 'STPLC_10';
	date_element: DATE;
	Processtime: STRING;
	TimerProcess: TON;
END_VAR
