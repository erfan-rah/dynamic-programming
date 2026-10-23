close all
clear
clc

%% Enter Number of Stages (N)
N = 2;

%% Available States and Control Law Values
states = (0:0.5:1.5)';
control = (-1:0.5:1)';

%% Initializing
x = inf(length(states),length(control));
u = x;
J = inf(length(states),length(control),N);
J_star = inf(length(states),1,N);
J_opt = inf(length(states),length(control),N);
u_star = J_star;

%% Solvation Loop
for k = N:-1:1
    for i = 1:size(x,1)
        for j = 1:size(x,2)
            if k == N
                temp = states(i) + control(j);
                if temp >= min(states) && temp <= max(states)
                    x(i,j) = temp;
                    u(i,j) = control(j);
                end
                %% Last Stage Cost Function
                J(i,j,k) = x(i,j)^2 + 2*u(i,j)^2;
            else
                %% Cost Function for Previous Stages
                J(i,j,k) = J_opt(i,j,k+1) + 2*u(i,j)^2;
            end
        end
        %% Optimal Solution (J_star) and Optimal Control Input (u_star)
        [J_star(i,:,:),ind_star(i,:,:)] = min(J(i,:,:));
        u_star(i,:,:) = u(i,ind_star(i));
    end
    for i = 1:size(x,1)
        for j = 1:size(x,2)
            for l = 1:size(x,1)
                if abs(x(i,j) - states(l)) < 1e-9
                    %% J_opt puts J_star values to corresponding arrays of state matrix (x)
                    J_opt(i,j,k) = J_star(l,:,k);
                end
            end
        end
    end
end
