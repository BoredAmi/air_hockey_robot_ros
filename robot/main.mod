MODULE Main_EGM

    VAR jointtarget startPosition:=[there is data on the robot i will not share it here for now];
    
    PROC Main()
        MoveAbsJ startPosition,v100,fine,tool0;
        WHILE TRUE DO
            run_egm;
        ENDWHILE           
    ENDPROC

ENDMODULE