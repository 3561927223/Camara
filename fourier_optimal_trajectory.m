function fourier_optimal_trajectory()
    % 基于傅里叶级数的动力学参数辨识最优轨迹生成（修正版）
    % 轨迹形式: q_j(t) = sum_{l=1}^L [a_{l,j}/(lωf) * sin(lωf t) - b_{l,j}/(lωf) * cos(lωf t)] + q0,j

    rng default; % 结果可复现

    % 1. 参数设置
    params = struct();
    params.nDOF = 7;                 % 7自由度机械臂
    params.nParams = 57;             % 待辨识基础参数数量
    params.T = 20;                   % 轨迹总时间(s)
    params.N = 1000;                 % 离散点数量
    params.dt = params.T/(params.N-1); % 与 linspace(0,T,N) 一致
    params.L = 3;                    % 傅里叶级数项数
    params.omega_f = 2*pi/params.T;  % 基频

    % 关节约束（Franka机械臂）
    params.q_min = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
    params.q_max = [ 2.8973; 1.7628; 2.8973; -0.0698; 2.8973; 3.7525; 2.8973];
    params.dq_min = [-2.1750; -2.1750; -2.1750; -2.1750; -2.6100; -2.6100; -2.6100];
    params.dq_max = [ 2.1750;  2.1750;  2.1750;  2.1750;  2.6100;  2.6100;  2.6100];
    params.ddq_min = -15*ones(params.nDOF,1);
    params.ddq_max =  15*ones(params.nDOF,1);

    % 初始偏移量（使第4关节围绕可行区间中点摆动）
    params.q0 = zeros(params.nDOF, 1);
    params.q0(4) = (params.q_min(4) + params.q_max(4)) / 2;

    % 2. 初始傅里叶系数（优化变量）
    nCoeffsPerDOF = 2 * params.L;        % 每个关节的系数数量（a_l和b_l）
    nVars = nCoeffsPerDOF * params.nDOF; % 总变量数量：2*L*7
    x0 = 0.1 * randn(nVars, 1);          % 初始系数（小随机值）

    % 3. 配置遗传算法选项
    popSize = 30;
    initPop = repmat(x0.', popSize, 1);
    if popSize > 1
        initPop(2:end,:) = initPop(2:end,:) + 0.01*randn(popSize-1, nVars);
    end

    options = optimoptions('ga', ...
        'PopulationSize', popSize, ...
        'MaxGenerations', 100, ...
        'MaxStallGenerations', 30, ...
        'CrossoverFraction', 0.8, ...
        'MutationFcn', @mutationadaptfeasible, ...
        'CrossoverFcn', @crossoverscattered, ...
        'SelectionFcn', @selectiontournament, ...
        'InitialPopulationMatrix', initPop, ...
        'Display', 'iter', ...
        'PlotFcn', @gaplotbestf ...
    );

    % 4. 执行遗传算法优化
    % 变量上下界：限制傅里叶系数范围（避免轨迹过大）
    lb = -0.1 * ones(nVars, 1);
    ub =  0.1 * ones(nVars, 1);

    [x_opt, fval] = ga( ...
        @(x) objective_function(x, params), ...  % 目标函数（最小化负logdet）
        nVars, ...                               % 优化变量数量
        [], [], [], [], ...                      % 无线性约束（A, b, Aeq, Beq）
        lb, ub, ...                              % 变量上下界
        @(x) constraint_function(x, params), ... % 非线性约束（关节限制）
        options ...                              % 遗传算法选项
    );

    %#ok<ASGLU> fval  % 抑制未使用变量告警

    % 5. 解析最优轨迹
    [t, q_opt, dq_opt, ddq_opt] = get_trajectory(x_opt, params);

    % 6. 计算Fisher信息矩阵行列式（评估优化效果）
    F_optimal = fisher_information_matrix(q_opt, dq_opt, ddq_opt, params);
    logdet_F = safe_logdet_spd(F_optimal);
    det_F_optimal = exp(logdet_F);
    fprintf('最优轨迹信息矩阵logdet(F): %.4f, det(F): %.4e\n', logdet_F, det_F_optimal);

    % 7. 可视化结果（显示所有7个关节的轨迹）
    visualize_results(t, q_opt, dq_opt, ddq_opt, params.nDOF);
end

function [t, q, dq, ddq] = get_trajectory(x, params)
    % 根据傅里叶系数计算轨迹、速度和加速度
    nDOF = params.nDOF;
    L = params.L;
    omega_f = params.omega_f;
    T = params.T;
    N = params.N;
    t = linspace(0, T, N);

    % 初始化
    q = zeros(nDOF, N);
    dq = zeros(nDOF, N);
    ddq = zeros(nDOF, N);

    % 解析傅里叶系数（按关节拆分）
    coeffs = reshape(x, 2*L, nDOF);  % 2L行（a1..aL, b1..bL），nDOF列（关节）

    for j = 1:nDOF
        a = coeffs(1:L, j);       % a_{l,j}系数
        b = coeffs(L+1:2*L, j);   % b_{l,j}系数

        % 角度轨迹 q_j(t)
        for l = 1:L
            q(j,:) = q(j,:) + (a(l)/(l*omega_f)) * sin(l*omega_f*t) ...
                            - (b(l)/(l*omega_f)) * cos(l*omega_f*t);
        end
        q(j,:) = q(j,:) + params.q0(j);

        % 速度 dq_j(t)
        for l = 1:L
            dq(j,:) = dq(j,:) + a(l)*cos(l*omega_f*t) + b(l)*sin(l*omega_f*t);
        end

        % 加速度 ddq_j(t)
        for l = 1:L
            ddq(j,:) = ddq(j,:) - l*omega_f*a(l)*sin(l*omega_f*t) ...
                                + l*omega_f*b(l)*cos(l*omega_f*t);
        end
    end
end

function [c, ceq] = constraint_function(x, params)
    % 约束函数：关节角度（必要时可扩展速度、加速度）
    [~, q, dq, ddq] = get_trajectory(x, params);
    nDOF = params.nDOF;
    c = [];
    step = 50;  % 约束检查降采样步长
    idx = 1:step:size(q,2);

    % 角度约束：q_min <= q <= q_max → q - q_max <=0 且 q_min - q <=0
    for j = 1:nDOF
        c = [c; (q(j,idx) - params.q_max(j)).'; ...
                (params.q_min(j) - q(j,idx)).'];
    end

    % 若需要启用速度/加速度限制，取消下方注释
    % for j = 1:nDOF
    %     c = [c; (dq(j,idx) - params.dq_max(j)).'; ...
    %             (params.dq_min(j) - dq(j,idx)).'];
    % end
    % for j = 1:nDOF
    %     c = [c; (ddq(j,idx) - params.ddq_max(j)).'; ...
    %             (params.ddq_min(j) - ddq(j,idx)).'];
    % end

    ceq = [];
end

function J = objective_function(x, params)
    % 目标函数：最大化 Fisher 信息矩阵行列式（通过最小化负logdet实现）
    [~, q, dq, ddq] = get_trajectory(x, params);
    F = fisher_information_matrix(q, dq, ddq, params);
    logdetF = safe_logdet_spd(F);
    J = -logdetF;  % 最小化负logdet = 最大化行列式
end

function F = fisher_information_matrix(q, dq, ddq, params)
    % 计算Fisher信息矩阵：F = sum(M' * M)，M为 7×57 回归矩阵
    nParams = params.nParams;   % 57
    N = params.N;
    F = zeros(nParams);
    step = 10;  % 降采样步长
    for k = 1:step:N
        % get_Regressor 期望 1×7 行向量，返回 1×(7*57) 展平回归矩阵
        Mflat = get_Regressor(q(:,k).', dq(:,k).', ddq(:,k).'); % 1×399

        % 基本校验
        expectedCols = params.nDOF * nParams; % 399
        if size(Mflat,2) ~= expectedCols
            error('get_Regressor 返回的列数为 %d，期望 %d (7*57)。', size(Mflat,2), expectedCols);
        end

        % 还原为 7×57（每57列为一个关节块）
        M = reshape(Mflat, nParams, params.nDOF).'; % 7×57

        % 累积信息矩阵
        F = F + M.' * M;
    end
end

function val = safe_logdet_spd(F)
    % 稳健 logdet 计算（Cholesky + 抖动）
    jitter = 1e-9;
    I = eye(size(F));
    for i = 1:8
        [R,p] = chol(F + jitter*I);
        if p == 0
            val = 2*sum(log(diag(R)));
            return
        end
        jitter = jitter * 10;
    end
    % 如果仍非SPD，返回极小 logdet（对应巨大的惩罚）
    val = -1e12;
end

function visualize_results(t, q, dq, ddq, nDOF)
    % 可视化所有关节的轨迹、速度、加速度
    figure('Name', '关节角度轨迹', 'Position', [100 100 1000 600]);
    for j = 1:nDOF
        subplot(ceil(nDOF/2), 2, j);
        plot(t, q(j,:), 'LineWidth', 1.2);
        xlabel('时间 (s)'); ylabel(['关节' num2str(j) '角度 (rad)']);
        grid on; box on;
    end
    sgtitle('最优关节角度轨迹');

    figure('Name', '关节速度轨迹', 'Position', [100 200 1000 600]);
    for j = 1:nDOF
        subplot(ceil(nDOF/2), 2, j);
        plot(t, dq(j,:), 'LineWidth', 1.2);
        xlabel('时间 (s)'); ylabel(['关节' num2str(j) '速度 (rad/s)']);
        grid on; box on;
    end
    sgtitle('最优关节速度轨迹');

    figure('Name', '关节加速度轨迹', 'Position', [100 300 1000 600]);
    for j = 1:nDOF
        subplot(ceil(nDOF/2), 2, j);
        plot(t, ddq(j,:), 'LineWidth', 1.2);
        xlabel('时间 (s)'); ylabel(['关节' num2str(j) '加速度 (rad/s^2)']);
        grid on; box on;
    end
    sgtitle('最优关节加速度轨迹');
end

