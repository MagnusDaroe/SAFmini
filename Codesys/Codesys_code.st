CASE state OF
	
	0: // init
	
	
	IO.xQA1_RIGHT := TRUE;	// Turns conveyor belt on
	IO.xQA1_SLOW := TRUE;
	IO.xMB20 := FALSE;		// Set gate to closed
	RFID.ClearError();		// Clear Rfid error
	TCPclient.Connect(sIP:= '172.20.66.80', uiPort:=24441); //Connect to server
	state := 1;
	
	1: // Resets values used later, and checks connection
	CorrectedID := 0;		//resets carrierID
	CarrierID := '';
	datetimestring := '';	//resets date string
	Processtime := '';		//resets processtime
	datetimeFB.xExecute := FALSE;	//resets get datetime command
	RfidData.uiCarrierID := 0;

	// Connects to server, if still not connected, and checks if the tcpClient is ready.
	IF NOT TCPclient.xConnected THEN
		TCPclient.Connect(sIP:= '172.20.66.80', uiPort:=24441);
	ELSIF TCPclient.xConnected AND TCPclient.xReady THEN
		state := 2;
	END_IF
	
	
	2: // Connects and checks if RFID is ready
	IF RFID.xReady THEN
		RFID.Connect(usiNodeId := 32, usiNetworkId := 0, usiChannel :=1);
		state := 4 ;
	END_IF
	
	
	4: //reads RFID tag	when arrived at the last sensor at the gate.
	IF IO.xBG21 = TRUE THEN		
		IF RFID.xReady THEN
			IF RfidData.uiCarrierID = 0 THEN
				RFID.ReadTag(uiStartAddress := 0, uiDataLength := SIZEOF(RfidData), pData := ADR(RfidData));
			ELSE
				state := 5;
			END_IF
		END_IF
	END_IF
		

	
	5: // corrects the rfid data, and makes it a string, and starts gathering the date of the PLC

	IF RFID.xReady THEN
		correctedID := dc_ecp.SwapWORD(RfidData.uiCarrierID); //swaps the data
		CarrierID := UINT_TO_STRING(correctedID); 
		datetimeFB.xExecute := TRUE; 
		state := 6;
	END_IF
	
	6: // Checks if the gathered date and time is ready, and extracts it.

	IF datetimeFB.xDone THEN
		datetime := datetimeFB.dtDateAndTime;
		datetimestring := DT_TO_STRING(datetime);
		IF datetimestring = '' THEN //if date is empty, check again.
			state := 6;
		ELSE
			state := 7;
		END_IF
		
	END_IF
	
	
	7: //create xml string
	
	XML := '<CARRIERDATA><PLCID>';
	XML := CONCAT(XML,PLCID);
	XML := CONCAT(XML,'</PLCID><DATEANDTIME>');
	XML := CONCAT(XML,datetimestring);
	XML := CONCAT(XML,'</DATEANDTIME><CARRIERID>');
	XML := CONCAT(XML,CarrierID);
	XML := CONCAT(XML,'</CARRIERID></CARRIERDATA>');
	state := 8;
	
	
	8: // sends xml data to server
	IF TCPclient.xReady THEN
		TCPclient.Send(pToSend:=ADR(XML), uiSizeToSend:=SIZEOF(XML));
		state := 9;
	END_IF
	
	9: // receive processing time from server
	IF TCPclient.xReady THEN
		IF Processtime = '' THEN // if processtime is nothing, recieve data from server.
			TCPclient.Receive(pToReceive:=ADR(Processtime), uiSizeToReceive:=SIZEOF(Processtime));
		ELSE // starts process timer.
			TimerProcess(IN:=TRUE, PT:= STRING_TO_TIME(ProcessTime));
			state := 10;
		END_IF
	END_IF

	
	10: // waits for the timer to has been passed, and turn on the conveyors again, and open the gate.
	IF TimerProcess.Q THEN
		TimerProcess(IN := FALSE);
		IO.xMB20 := TRUE;
		state := 11;
	END_IF
	
	11: // close gate behind the carrier, and return to state 1, which enables the next carrier to be processed.
	IF IO.xBG21 = FALSE THEN
		IO.xMB20 := FALSE;
		state := 1;
	END_IF
END_CASE

TCPclient();
RFID();
DatetimeFB();
TimerProcess();
