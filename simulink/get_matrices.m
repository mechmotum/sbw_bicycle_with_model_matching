function [ M_pp, M_pd, M_dp, M_dd, ...
           K0_pp, K0_pd, K0_dp, K0_dd, ...
           K2_pd, K2_dd, ...
           C1_pd, C1_dp, C1_dd ] = get_matrices(bikePar)
% FIND_MATRICES function calculates the Whipple-Carvallo model matrices.
%   Mx4, K0x4, K2x2, C1x3 = FIND_MATRICES(bikePar, velocity)

%% Setting bicycle parameters
    % General parameters
    w = bikePar.w;
    c = bikePar.c;
    lambda = bikePar.lambda;
    % Rear wheel
    r_R = bikePar.rR;
    m_R = bikePar.mR;
    I_Rxx = bikePar.IRxx;
    I_Ryy = bikePar.IRyy;
    I_Rzz = bikePar.IRzz;
    % Rider body and frame
    x_B = bikePar.xB;
    z_B = bikePar.zB;
    m_B = bikePar.mB;
    I_Bxx = bikePar.IBxx;
    I_Byy = bikePar.IByy;
    I_Bzz = bikePar.IBzz;
    I_Bxz = bikePar.IBxz;
    % Handlebar and fork
    x_H = bikePar.xH;
    z_H = bikePar.zH;
    m_H = bikePar.mH;
    I_Hxx = bikePar.IHxx;
    I_Hyy = bikePar.IHyy;
    I_Hzz = bikePar.IHzz;
    I_Hxz = bikePar.IHxz;
    % Front wheel
    r_F = bikePar.rF;
    m_F = bikePar.mF;
    I_Fxx = bikePar.IFxx;
    I_Fyy = bikePar.IFyy;
    I_Fzz = bikePar.IFzz;
    
%% Intermediate equations
    % Equations for the whole bicycle
    m_T = m_R + m_B + m_H + m_F;
    x_T = (x_B * m_B + x_H * m_H + w * m_F) / m_T;
    z_T = (-r_R * m_R + z_B * m_B + z_H * m_H - r_F*m_F) / m_T;
    I_Txx = I_Rxx + I_Bxx + I_Hxx + I_Fxx + ...
        m_R * r_R^2 + m_B * z_B^2 + m_H * z_H^2 + m_F * r_F^2;
    I_Txz = I_Bxz + I_Hxz - m_B * x_B * z_B - m_H * x_H * z_H + m_F * w * r_F;
    I_Tzz = I_Rzz + I_Bzz + I_Hzz + I_Fzz + ...
        m_B * x_B^2 + m_H * x_H^2 + m_F * w^2;
    % Equations for the front wheel and handlebar assembly
    m_A = m_H + m_F;
    x_A = (x_H * m_H + w * m_F) / m_A;
    z_A = (z_H * m_H - r_F * m_F) / m_A;
    I_Axx = I_Hxx + I_Fxx + m_H * (z_H - z_A)^2 + m_F * (r_F + z_A)^2;
    I_Axz = I_Hxz - m_H * (x_H - x_A) * (z_H - z_A) + m_F * (w - x_A) * (r_F + z_A);
    I_Azz = I_Hzz + I_Fzz + m_H * (x_H - x_A)^2 + m_F * (w - x_A)^2;
    % Equations for the steering axis
    u_A = (x_A - w - c) * cos(lambda) - z_A * sin(lambda);
    I_All = m_A * u_A^2 + I_Axx * sin(lambda)^2 + ...
        2 * I_Axz * sin(lambda) * cos(lambda) + I_Azz * cos(lambda)^2;
    I_Alx = -m_A * u_A * z_A + I_Axx * sin(lambda) + I_Axz * cos(lambda);
    I_Alz = m_A * u_A * x_A + I_Axz * sin(lambda) + I_Azz * cos(lambda);
    mu = (c / w) * cos(lambda);
    % Momenta
    S_R = I_Ryy / r_R;
    S_F = I_Fyy / r_F;
    S_T = S_R + S_F;
    S_A = m_A * u_A + mu * m_T * x_T;
    
%% Populating matrices
    % Mass matrix
    % M = [M_pp, M_pd;
    %      M_dp, M_dd];
    M_pp = I_Txx;
    M_pd = I_Alx + mu * I_Txz;
    M_dp = I_Alx + mu * I_Txz;
    M_dd = I_All + 2 * mu * I_Alz + mu^2 * I_Tzz;
    % Stiffness due to gravity matrix
    % K0 = [K0_pp, K0_pd;
    %       K0_dp, K0_dd];
    K0_pp = m_T * z_T;
    K0_pd = -S_A;
    K0_dp = -S_A;
    K0_dd = -S_A * sin(lambda);
    % Stiffness due to velocity matrix
    % K2 = [0, K2_pd;
    %       0, K2_dd];
    K2_pd = ((S_T - m_T * z_T) / w) * cos(lambda);
    K2_dd = ((S_A + S_F * sin(lambda)) / w) * cos(lambda);
    % Damping-like matrix
    % C1 = [0,     C1_pd;
    %       C1_dp, C1_dd];
    C1_pd = mu * S_T + S_F * cos(lambda) + ...
        (I_Txz / w) * cos(lambda) - mu * m_T * z_T;
    C1_dp = -(mu * S_T + S_F * cos(lambda));
    C1_dd = (I_Alz / w) * cos(lambda) + mu * (S_A + (I_Tzz / w) * cos(lambda));

end