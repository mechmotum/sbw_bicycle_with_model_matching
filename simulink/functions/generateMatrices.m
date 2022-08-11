function generateMatrices(options, discreteStateSpace)
% Generates matrices needed for MPC/QP from given options and bicycle model

%% Extract needed options
nX = options.nX; nSH = options.nSH; nU = options.nU;

%% Pre-allocate arrays
A_full = zeros(nX * nSH, nX);
B_full = zeros(nX * nSH, nU * (nSH - 2));
C_full = zeros(nX * nSH, nU);
R_full = zeros(nU * (nSH - 2));
ub_x_full = zeros(nX * nSH, 1);
ub_u_full = zeros(nU * (nSH - 2), 1);
lb_x_full = ub_x_full;
lb_u_full = ub_u_full;

%% Populate arrays
for i= 1 : nSH
    % A
    A_full(1+nX*(i-1) : nX*i, :) = discreteStateSpace.A^(i-1);
    % B
    for j=1:(i-2)
        B_full(1+nX*(i-1) : nX*i, 1+nU*(j-1) : nU*j) = ...
            discreteStateSpace.A^(i-j-2) * discreteStateSpace.B;
    end
    % C
    if i ~= 1
        C_full(1+nX*(i-1) : nX*i, :) = ...
            discreteStateSpace.A^(i-2) * discreteStateSpace.B;
    end
    % R
    if i ~= 1 && i ~= nSH
        R_full(1+nU*(i-2) : nU*(i-1), 1+nU*(i-2) : nU*(i-1)) = options.R;
    end
    % ub_x lb_x
    ub_x_full(1+nX*(i-1) : nX*i, :) = options.ub_x;
    lb_x_full(1+nX*(i-1) : nX*i, :) = options.lb_x;
    % ub_u lb_u
    if i ~= 1 && i ~= nSH
        ub_u_full(1+nU*(i-2) : nU*(i-1), :) = options.ub_u;
        lb_u_full(1+nU*(i-2) : nU*(i-1), :) = options.lb_u;
    end
end

R_full_zeros = zeros(size(R_full));

%% Save to file
cd("." + filesep + "mat_files");
save matrices.mat A_full B_full C_full R_full R_full_zeros ...
     ub_x_full ub_u_full lb_x_full lb_u_full;
cd("..");

end