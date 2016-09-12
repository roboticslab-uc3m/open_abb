MODULE SERVER

!////////////////
!GLOBAL VARIABLES
!////////////////

!//Robot configuration
PERS tooldata currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];   
PERS speeddata currentSpeed;
PERS zonedata currentZone;

PERS tooldata asus:=[TRUE,[[0,0,1000],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
PERS tooldata pinzaTextiles:=[TRUE,[[48.6251,8.30102,269.235],[1,0,0,0]],[4,[0,0,-100],[1,0,0,0],0,0,0]];
PERS tooldata asusTextiles:=[TRUE,[[121.94,54.5219,84.6058],[1,0,0,0]],[4,[0,0,-100],[1,0,0,0],0,0,0]];

!// Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

!// Mutex between logger and changing the tool and work objects
PERS bool frameMutex:=FALSE;

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR num params{10};
VAR num nParams;

PERS string ipController:= "192.168.125.1"; !robot default IP
!PERS string ipController:= "127.0.0.1"; !local IP for testing in simulation
PERS num serverPort:= 5000;

!//Motion of the robot
VAR robtarget cartesianTarget;
VAR jointtarget jointsTarget;
VAR bool moveCompleted; !Set to true after finishing a Move instruction.

!//Buffered move variables
CONST num MAX_BUFFER := 512;
VAR num BUFFER_POS := 0;
VAR robtarget bufferTargets{MAX_BUFFER};
VAR speeddata bufferSpeeds{MAX_BUFFER};

!//External axis position variables
VAR extjoint externalAxis;

!//Circular move buffer
VAR robtarget circPoint;

!//Correct Instruction Execution and possible return values
VAR num ok;
CONST num SERVER_BAD_MSG :=  0;
CONST num SERVER_OK := 1;



	
!////////////////
!LOCAL METHODS
!////////////////

!//Method to parse the message received from a PC
!// If correct message, loads values on:
!// - instructionCode.
!// - nParams: Number of received parameters.
!// - params{nParams}: Vector of received params.
PROC ParseMsg(string msg)
    !//Local variables
    VAR bool auxOk;
    VAR num ind:=1;
    VAR num newInd;
    VAR num length;
    VAR num indParam:=1;
    VAR string subString;
    VAR bool end := FALSE;
	
    !//Find the end character
    length := StrMatch(msg,1,"#");
    IF length > StrLen(msg) THEN
        !//Corrupt message
        nParams := -1;
    ELSE
        !//Read Instruction code
        newInd := StrMatch(msg,ind," ") + 1;
        subString := StrPart(msg,ind,newInd - ind - 1);
        auxOk:= StrToVal(subString, instructionCode);
        IF auxOk = FALSE THEN
            !//Impossible to read instruction code
            nParams := -1;
        ELSE
            ind := newInd;
            !//Read all instruction parameters (maximum of 8)
            WHILE end = FALSE DO
                newInd := StrMatch(msg,ind," ") + 1;
                IF newInd > length THEN
                    end := TRUE;
                ELSE
                    subString := StrPart(msg,ind,newInd - ind - 1);
                    auxOk := StrToVal(subString, params{indParam});
                    indParam := indParam + 1;
                    ind := newInd;
                ENDIF	   
            ENDWHILE
            nParams:= indParam - 1;
        ENDIF
    ENDIF
ENDPROC


!//Handshake between server and client:
!// - Creates socket.
!// - Waits for incoming TCP connection.
PROC ServerCreateAndConnect(string ip, num port)
    VAR string clientIP;
	
    SocketCreate serverSocket;
    SocketBind serverSocket, ip, port;
    SocketListen serverSocket;
    TPWrite "SERVER: Server waiting for incoming connections ...";
    WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
        SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
        IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
            TPWrite "SERVER: Problem serving an incoming connection.";
            TPWrite "SERVER: Try reconnecting.";
        ENDIF
        !//Wait 0.5 seconds for the next reconnection
        WaitTime 0.5;
    ENDWHILE
    TPWrite "SERVER: Connected to IP " + clientIP;
ENDPROC


!//Parameter initialization
!// Loads default values for
!// - Tool.
!// - WorkObject.
!// - Zone.
!// - Speed.
PROC Initialize()
    currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
    currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    currentSpeed := [100, 50, 0, 0];
    currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	
	!Find the current external axis values so they don't move when we start
	jointsTarget := CJointT();
	externalAxis := jointsTarget.extax;
ENDPROC


!////////////////////////
!//SERVER: Main procedure
!////////////////////////
PROC main()
    !//Local variables
    VAR string receivedString;   !//Received string
    VAR string sendString;       !//Reply string
    VAR string addString;        !//String to add to the reply.
    VAR bool connected;          !//Client connected
    VAR bool reconnected;        !//Drop and reconnection happened during serving a command
    VAR robtarget cartesianPose;
    VAR jointtarget jointsPose;
    			
    !//Motion configuration
    ConfL \Off;
    SingArea \Wrist;
    moveCompleted:= TRUE;
	
    !//Initialization of WorkObject, Tool, Speed and Zone
    Initialize;

    !//Socket connection
    connected:=FALSE;
    ServerCreateAndConnect ipController,serverPort;	
    connected:=TRUE;
    
    !//Server Loop
    WHILE TRUE DO
        !//Initialization of program flow variables
        ok:=SERVER_OK;              !//Correctness of executed instruction.
        reconnected:=FALSE;         !//Has communication dropped after receiving a command?
        addString := "";            

        !//Wait for a command
        SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
        ParseMsg receivedString;
	
        !//Execution of the command
        TEST instructionCode
            CASE 0: !Ping
                IF nParams = 0 THEN
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 1: !Cartesian Move
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [0,0,0,0],
                                       externalAxis];
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    moveCompleted := TRUE;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF	
				
            CASE 2: !Joint Move
                IF nParams = 6 THEN
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}], externalAxis];
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    moveCompleted := TRUE;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams = 0 THEN
                    cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);		
                    addString := NumToStr(cartesianPose.trans.x,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q1,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q2,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q3,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q4,3); !End of string	
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 4: !Get Joint Coordinates
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    addString := NumToStr(jointsPose.robax.rax_1,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_2,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_3,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_4,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_5,2) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_6,2); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
			CASE 5: !Get external axis positions
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    addString := StrPart(NumToStr(jointsTarget.extax.eax_a, 2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_b,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_c,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_d,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_e,2),1,8) + " ";
                    addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_f,2),1,8); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF	
		
            CASE 6: !Set Tool
                IF nParams = 7 THEN
		   WHILE (frameMutex) DO
		        WaitTime .01; !// If the frame is being used by logger, wait here
		   ENDWHILE
		frameMutex:= TRUE;
                    currentTool.tframe.trans.x:=params{1};
                    currentTool.tframe.trans.y:=params{2};
                    currentTool.tframe.trans.z:=params{3};
                    currentTool.tframe.rot.q1:=params{4};
                    currentTool.tframe.rot.q2:=params{5};
                    currentTool.tframe.rot.q3:=params{6};
                    currentTool.tframe.rot.q4:=params{7};
                    ok := SERVER_OK;
		    frameMutex:= FALSE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 7: !Set Work Object
                IF nParams = 7 THEN
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 8: !Set Speed of the Robot
                IF nParams = 4 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    currentSpeed.v_leax:=params{3};
                    currentSpeed.v_reax:=params{4};
                    ok := SERVER_OK;
                ELSEIF nParams = 2 THEN
					currentSpeed.v_tcp:=params{1};
					currentSpeed.v_ori:=params{2};
					ok := SERVER_OK;
				ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 9: !Set zone data
                IF nParams = 4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep := TRUE;
                        currentZone.pzone_tcp := 0.0;
                        currentZone.pzone_ori := 0.0;
                        currentZone.zone_ori := 0.0;
                    ELSE
                        currentZone.finep := FALSE;
                        currentZone.pzone_tcp := params{2};
                        currentZone.pzone_ori := params{3};
                        currentZone.zone_ori := params{4};
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 30: !Add Cartesian Coordinates to buffer
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    IF BUFFER_POS < MAX_BUFFER THEN
                        BUFFER_POS := BUFFER_POS + 1;
                        bufferTargets{BUFFER_POS} := cartesianTarget;
                        bufferSpeeds{BUFFER_POS} := currentSpeed;
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 31: !Clear Cartesian Buffer
                IF nParams = 0 THEN
                    BUFFER_POS := 0;	
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 32: !Get Buffer Size)
                IF nParams = 0 THEN
                    addString := NumToStr(BUFFER_POS,2);
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 33: !Execute moves in cartesianBuffer as linear moves
                IF nParams = 0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO 
                        MoveL bufferTargets{i}, bufferSpeeds{i}, currentZone, currentTool \WObj:=currentWobj ;
                    ENDFOR			
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 34: !External Axis move
                IF nParams = 6 THEN
                    externalAxis :=[params{1},params{2},params{3},params{4},params{5},params{6}];
                    jointsTarget := CJointT();
                    jointsTarget.extax := externalAxis;
                    ok := SERVER_OK;
                    moveCompleted := FALSE;
                    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    moveCompleted := TRUE;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 35: !Specify circPoint for circular move, and then wait on toPoint
                IF nParams = 7 THEN
                    circPoint :=[[params{1},params{2},params{3}],
                                [params{4},params{5},params{6},params{7}],
                                [0,0,0,0],
                                externalAxis];
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 36: !specify toPoint, and use circPoint specified previously
                IF nParams = 7 THEN
                    cartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    MoveC circPoint, cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 80:
                scan;
                ok := SERVER_OK;
                
            CASE 90:
                IF nParams = 5 THEN
                    pickAndPlace params{1}, params{2}, params{3}, params{4}, params{5};
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                
            CASE 97:
                IF nParams = 1 THEN
                    IF params{1} = 1 THEN
                        SetDO DO10_2, high;
                    ELSE
                        SetDO DO10_2, low;
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 98: !returns current robot info: serial number, robotware version, and robot type
                IF nParams = 0 THEN
                    addString := GetSysInfo(\SerialNo) + "*";
                    addString := addString + GetSysInfo(\SWVersion) + "*";
                    addString := addString + GetSysInfo(\RobotType);
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF
			
            CASE 99: !Close Connection
                IF nParams = 0 THEN
                    TPWrite "SERVER: Client has closed connection.";
                    connected := FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;

                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected := TRUE;
                    reconnected := TRUE;
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF
            DEFAULT:
                TPWrite "SERVER: Illegal instruction code";
                ok := SERVER_BAD_MSG;
        ENDTEST
		
        !Compose the acknowledge string to send back to the client
        IF connected = TRUE THEN
            IF reconnected = FALSE THEN
			    IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
				    sendString := NumToStr(instructionCode,0);
                    sendString := sendString + " " + NumToStr(ok,0);
                    sendString := sendString + " " + addString;
                    SocketSend clientSocket \Str:=sendString;
			    ENDIF
            ENDIF
        ENDIF
    ENDWHILE

ERROR (LONG_JMP_ALL_ERR)
    TPWrite "SERVER: ------";
    TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
    TEST ERRNO
        CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: Lost connection to the client.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= TRUE;
            connected:= TRUE;
            RETRY; 
        DEFAULT:
            TPWrite "SERVER: Unknown error.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= TRUE;
            connected:= TRUE;
            RETRY;
    ENDTEST
ENDPROC

PROC scan()
	CONST robtarget topLeft:=[[1107.84,-282.81,60.31],[0.0495452,0.00703024,0.998742,0.00318766],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget topRight:=[[1112.08,206.07,63.52],[0.0495474,0.00702667,0.998742,0.00318459],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget bottomRight:=[[1511.65,200.45,103.25],[0.0495497,0.00702489,0.998742,0.00319618],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget bottomLeft:=[[1505.82,-223.65,100.26],[0.0495496,0.00702869,0.998742,0.00318613],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center:=[[1359.84,-77.90,86.56],[0.049549,0.00703216,0.998742,0.00318921],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget p10:=[[1287.37,-15.44,160.34],[0.202251,-0.0227523,0.979061,0.00404474],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center10:=[[1362.89,-21.85,85.89],[0.31284,-0.011903,0.949699,0.00783877],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center20:=[[1362.90,-21.85,85.90],[0.174129,-0.126191,0.925385,0.312118],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center30:=[[1362.89,-21.84,85.90],[0.0139594,0.137217,-0.990391,-0.0100989],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center40:=[[1362.90,-21.84,85.89],[0.0169112,-0.129144,0.907029,-0.400417],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center50:=[[1374.73,-15.03,96.89],[0.344279,-0.24864,0.837086,-0.344871],[0,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center60:=[[1374.73,-15.02,96.89],[0.34428,-0.248643,0.837085,-0.344871],[0,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center70:=[[1287.63,-30.37,135.41],[0.0904054,0.0316452,0.995401,0.00135333],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center80:=[[1287.62,-30.37,135.40],[0.222889,0.722363,-0.615767,0.222132],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center90:=[[1287.63,-30.36,135.40],[0.41068,-0.607237,0.637932,0.2359],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center100:=[[1331.89,-66.31,179.81],[0.471784,-0.387667,0.79185,0.0103615],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget center110:=[[1331.90,-66.31,179.82],[0.471787,-0.387671,0.791847,0.0103581],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST speeddata speed1:=[100,20,5000,1000];    
	MoveJ topLeft, speed1, z50, asus;
	MoveJ topRight, speed1, z50, asus;
	MoveJ bottomRight, speed1, z50, asus;
	MoveJ bottomLeft, speed1, z50, asus;
	MoveJ center, speed1, z50, asus;
	MoveJ center10, speed1, z50, asus;
	MoveJ center20, speed1, z50, asus;
	MoveJ center30, speed1, z50, asus;
	MoveJ center40, speed1, z50, asus;
	MoveJ center50, speed1, z50, asus;
	MoveJ center, speed1, z50, asus;
	MoveJ center70, speed1, z50, asus;
	MoveJ center80, speed1, z50, asus;
	MoveJ center90, speed1, z50, asus;
	MoveJ center100, speed1, z50, asus;
	MoveJ center, speed1, z50, asus;
ENDPROC

PROC pickAndPlace(num pPickDownX, num pPickDownY, num pPlaceDownX, num pPlaceDownY, num heightFromPickDown)
	CONST robtarget pReposo:=[[1000,0,500],[0.04955,0.00701,0.99874,0.003191],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	VAR robtarget pPickDown:=[[0,0,176],[0.04955,0.00701,0.99874,0.003191],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR robtarget pPlaceDown:=[[0,0,200],[0.04955,0.00701,0.99874,0.003191],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    !CONST num heightFromPickDown := 120; ! was 175
    pPickDown.trans.x := pPickDownX;
    pPickDown.trans.y := pPickDownY;
    pPlaceDown.trans.x := pPlaceDownX;
    pPlaceDown.trans.y := pPlaceDownY;
    SetDO DO10_2, high;
    !ConfJ\Off;
    !ConfL\Off;
    MoveJ pReposo, v100, z50, pinzaTextiles;
	MoveJ Offs(pPickDown,0,0,heightFromPickDown), v100, z50, pinzaTextiles;
	MoveL pPickDown, v50, fine, pinzaTextiles;
    WaitTime \InPos, 0.1;
    SetDO DO10_2, low;
    WaitTime 1;
    MoveL Offs(pPickDown,0,0,heightFromPickDown), v50, fine, pinzaTextiles;
	MoveJ Offs(pPlaceDown,0,0,heightFromPickDown-25), v100, z50, pinzaTextiles;
    MoveL pPlaceDown, v50, fine, pinzaTextiles;
    WaitTime \InPos, 0.1;
    SetDO DO10_2, high;
    WaitTime 1;
    MoveL Offs(pPlaceDown,0,0,heightFromPickDown-25), v50, fine, pinzaTextiles;
    MoveJ pReposo, v100, z50, pinzaTextiles;
    SetDO DO10_2, low;
ENDPROC
    
ENDMODULE