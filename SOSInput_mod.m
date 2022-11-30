function [X_com_pitch,X_com_roll,X_com_pitch_pn,X_com_roll_pn]=SOSInput_mod(Tpause, Ts, TendXc)

% Develop SOS input per the guidance in "Piloted Simulation Evaluation of
% Tracking MTEs for the Assessment of High-Speed Handling Qualities
  %- Inputs -%
   % Tpause : wait time to begin 
   % Ts     : sample time
   % TendXc : score time (60 s)

Nn = [3 5 8 13 21 34 55];
fn = Nn/60;

wn = fn*2*pi;
wn_pn = [0.524 0.838 1.361 2.199 3.561 5.760 9.32];

% Precision, Aggressive
Amp_pitch_deg = [2.493 1.793 1.216 0.776 0.487 0.303 0.188];
Amp_roll_deg = [11.814 7.854 5.104 3.190 2.002 1.232 0.770];

% Precision, Non-Aggressive
Amp_pitch_deg_pn = [2.951 -1.798 0.776 0.304 -0.116 -0.044 0.017];
Amp_roll_deg_pn = [11.394 -8.29 4.397 1.813 -0.700 -0.268 0.102];


t_c = [0:Ts:TendXc];

rn = 180*pi/180*rand(1,length(Amp_pitch_deg));

% Preallocate storage
X_pitch    = zeros(length(Amp_pitch_deg), length(t_c));
X_roll     = zeros(length(Amp_pitch_deg), length(t_c));
X_pitch_pn = zeros(length(Amp_pitch_deg), length(t_c));
X_roll_pn  = zeros(length(Amp_pitch_deg), length(t_c));


for ii =1:length(Amp_pitch_deg)
       if mod(ii,2)==0
           s=-1;
       else
           s=1;
       end
        X_pitch(ii,:) = s*Amp_pitch_deg(ii)*(sin(wn(ii)*t_c+rn(ii)));
        X_roll(ii,:) = s*Amp_roll_deg(ii)*(sin(wn(ii)*t_c+rn(ii)));
        
        X_pitch_pn(ii,:) = Amp_pitch_deg_pn(ii)*(sin(wn_pn(ii)*t_c+rn(ii)));
        X_roll_pn(ii,:) = Amp_roll_deg_pn(ii)*(sin(wn_pn(ii)*t_c+rn(ii)));
end


Xc_pitch = sum(X_pitch);
Xc_roll = sum(X_roll);

Xc_pitch_pn = sum(X_pitch_pn);
Xc_roll_pn = sum(X_roll_pn);

t_p=[0:Ts:Tpause]; % t_p Paused time vector for Xc = 0 % 
t_sig=[t_p, t_c+t_p(length(t_p))];

X_p = [zeros(size(t_p))];   % X_p  = 0 %
X_r = [zeros(size(t_p))];
X_p_pn =[zeros(size(t_p))];
X_r_pn = [zeros(size(t_p))];

X_sig_pitch=[X_p, Xc_pitch];    % new signal with concat zero and signal, Xc
X_sig_roll=[X_r,Xc_roll];
X_sig_pitch_pn=[X_p_pn, Xc_pitch_pn];   
X_sig_roll_pn=[X_r_pn,Xc_roll_pn];


X_com_pitch=[t_sig;X_sig_pitch]';
X_com_roll=[t_sig;X_sig_roll]';
X_com_pitch_pn=[t_sig;X_sig_pitch_pn]';
X_com_roll_pn=[t_sig;X_sig_roll_pn]';

return