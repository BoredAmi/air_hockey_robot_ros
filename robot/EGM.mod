MODULE EGM_Controls
VAR egmident egmID1;
VAR egmstate egmSt1;
CONST egm_minmax lin_mm := [-1.0, 1.0];

CONST pose corr_frame := [[0,0,0],[1,0,0,0]];
CONST pose sensor_frame := [[0,0,0],[1,0,0,0]];

PROC run_egm()
    AccSet 100, 100 \FinePointRamp:=100;
    VelSet 100, 5000;
    MotionSup \Off;
    EGMReset egmID1;
    EGMGetId egmID1;


egmSt1:=EGMGetState(egmID1);
TPWrite "EGM state: "\Num:=egmSt1;
IF egmSt1 <= EGM_STATE_CONNECTED THEN
EGMSetupUC ROB_1, egmID1, "default", "Rpi4b:" \Pose \CommTimeout:=100;
egmSt1:=EGMGetState(egmID1);
IF egmSt1 = EGM_STATE_CONNECTED THEN
TPWrite "ROB_1 CONNECTED!";
ENDIF
ENDIF

    EGMActPose egmID1 \Tool:=tool0 \WObj:=wobj0,
               corr_frame, EGM_FRAME_WOBJ,
               sensor_frame, EGM_FRAME_WOBJ
               \x:=lin_mm \y:=lin_mm \z:=lin_mm \SampleRate:=4 \maxspeeddeviation:=1000;

    EGMRunPose egmID1, EGM_STOP_HOLD \x \y \z \CondTime:=0.01 \RampInTime:=0.001 \RampOutTime:=0.001 \PosCorrGain:=1.0;

    egmSt1:=EGMGetState(egmID1);
    IF egmSt1=EGM_STATE_CONNECTED THEN
        TPWrite "Convergence condition fulfilled.. Resetting EGM.";
        EGMReset egmID1;
    ENDIF
ENDPROC


ENDMODULE
