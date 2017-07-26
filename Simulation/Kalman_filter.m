function [ Xn, Pn, A, B, H, Q, R ] = Kalman_filter( Un, Zn, Xp, Pp, A, B, H, Q, R )

    %% Vectors 
    %  Un   Control vector
    %  Zn   Measurement vector
    %  Xn   Estimated current state
    %  Pn   Estimated average error
    %  X    Predicted current state
    %  P    Predicted current covariance
    %  Xp   Previuos state
    %  Pp   Previuos average error
    %  Y    Innovation
    %  S    Innovation covariance
    %  K    Kalman gain
    %  A    State transition matrix
    %  B    Control matrix
    %  H	Observation matrix
    %  Q	Estimated process error covariance
    %  R	Estimated measurement error covariance
    
    %% Prediction
    X = A * Xp + B * Un;
    P = A * Pp * A.' + Q;
    
    %% Observation
    Y = Zn - H * X;
    S = H * P * H.' + R;
    
    %% Update step
    K = P * H.' * inv(S);
    Xn = X + K * Y;
    Pn = (eye(size(P)) - K * H) * P;
    
end

