% loiter input parameter
% W = W - W_cargo;
LoiterTime = 3600/2; % Loiter time (s)
% end loiter input parameter

% You can find the fuel used in (optimal) loiter by plugging LoiterTime
% into the appropriate endurance equation. W1 would be your
% (Wfuel_after_cruise+Wzerofuel). Then solve for W2, which is
% equivalent to (Wfuel_after_loiter+Wzerofuel)
% Wfuel_after_loiter = ; % (N)

v_loiter = sqrt(2 * W * sqrt(K / (3 * CD0)) / (rhocruise * Swing)); % Get a typical loiter velocity by finding the velocity that
% Gives you the optimal loiter time at the average of W1 and W2.

q_loiter =  0.5 * rhocruise * v_loiter^2;
Tr_loiter = q_loiter * Swing * CD0 + K * W^2 / (q_loiter * Swing); % Thrust required for steady level flight at cruise
Pr_loiter = Tr_loiter * v_loiter; % Power Required in cruise

E_loiter = (Pr_loiter * LoiterTime) / (eta_pr * eta_mo);
E = E - E_loiter;

% Also a good idea to check that your v_loiter > v_stall
v_stall_loiter = sqrt(2 * W / (rhocruise * Swing * CLmax)); % stall speed at loiter altitude/avg. weight

assert(v_loiter >= v_stall_loiter); % will fail if v_loiter < v_stall

W = W - W_cargo;

fprintf(1,'%% loiter output results\n');
% fprintf(1,'Wfuel_after_loiter=%.4g; %% Weight of fuel remaining after loiter (N)\n',Wfuel_after_loiter);
fprintf(1,'E_after_loiter=%.4g; %% Weight of fuel remaining after loiter (N)\n',E);
fprintf(1,'v_loiter=%.4g; %% averaged loiter velocity (m/s)\n',v_loiter);
fprintf(1,'v_stall_loiter=%.4g; %% stall speed at loiter altitude (m/s)\n',v_stall_loiter);
fprintf(1,'%% end loiter output results\n\n');