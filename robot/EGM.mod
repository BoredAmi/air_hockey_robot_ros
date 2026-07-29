ODULE EGM_FAST_CORRECTED
VAR egmident egmID1;
VAR egmstate egmSt1;
CONST egm_minmax egm_minmax1:=[-0.5,0.5];
PROC run_egm()
    AccSet 100, 100 \FinePointRamp:=100;
    VelSet 100, 5000;
    MotionSup \Off;
    EGMReset egmID1;
    EGMGetId egmID1;


egmSt1:=EGMGetState(egmID1);
TPWrite "EGM state: "\Num:=egmSt1;
IF egmSt1 <= EGM_STATE_CONNECTED THEN
EGMSetupUC ROB_1, egmID1, "default", "Rpi4b:" \Joint\ CommTimeout:=100;
egmSt1:=EGMGetState(egmID1);
IF egmSt1 = EGM_STATE_CONNECTED THEN
TPWrite "ROB_1 CONNECTED!";
ENDIF
ENDIF

    EGMActJoint egmID1\J1:=egm_minmax1\SampleRate:=8\MaxSpeedDeviation:=500000;

    EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \CondTime:=0.01 \RampInTime:=0.001 \RampOutTime:=0.001 \PosCorrGain:=1.0;

    egmSt1:=EGMGetState(egmID1);
    IF egmSt1=EGM_STATE_CONNECTED THEN
        TPWrite "Convergence condition fulfilled.. Resetting EGM.";
        EGMReset egmID1;
    ENDIF
ENDPROC


ENDMODULE