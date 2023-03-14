function [controller,input] = PID_attitude_step(reference, measure, controller)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PID controller for attitude (roll, pitch, yaw).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    bi = controller.Dt*controller.Ki;
    ad = controller.Tf/(controller.Tf+controller.Dt);
    bd = controller.Kd/(controller.Tf+controller.Dt);
        
    P = controller.Kp*angdiff(measure,reference);
    D = ad*controller.D +bd*(angdiff(measure,reference)-controller.Old_Error);
    
    % D Filter or anti-windup
    if (D < controller.Min_Input)
        D = controller.Min_Input;
    else if (D > controller.Max_Input)
            D = controller.Max_Input;
        end
    end
    
    % Anti-windup
    if (controller.Old_Input > controller.Max_Input)
        I = controller.Old_Input;
    else if (controller.Old_Input < controller.Min_Input)
            I = controller.Old_Input;
        else
            I = controller.I + bi*(angdiff(measure,reference));
        end
    end
    input = P + D + I;
    
    
    if (input < controller.Min_Input)
        input = controller.Min_Input;
    else if (input > controller.Max_Input)
            input = controller.Max_Input;
        end
    end
    
    controller.P = P;
    controller.I = I;
    controller.D = D;
    controller.Old_Error = angdiff(measure,reference);
    controller.Old_Output = measure;
    controller.Old_Input = input;
    
end
