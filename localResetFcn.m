function in = localResetFcn(in)
    % in: Simulink SimulationInput 객체
    
    % --- 시나리오 랜덤 선택 ---
    % 1: Soft Start (0V, 0W 시작 -> 전압 올리기)
    % 2: Steady State (이미 전압이 뜬 상태, 500W 부하)
    scenario_type = randi([1, 2]); 
    
    % --- 랜덤 노이즈 설정 ---
    noise_v = (rand - 0.5) * 2; % -1V ~ +1V 랜덤 오차
    
    if scenario_type == 1
        % [시나리오 1: 0W 기동]
        % 전압 0에서 시작, 부하 0, 목표 50V
        V_init = 0;
        I_init = 0;
        Pcpl_val = 0;      % 부하 없음
        V_ref_val = 50;    % 목표는 50V
        
    else
        % [시나리오 2: 500W 부하 상태 학습]
        % 중요: 0V가 아니라 45~55V 사이에서 시작해야 함!
        % 부하가 이미 걸려있으므로, 그에 맞는 인덕터 전류도 초기화해줘야 학습이 빠름
        
        V_ref_val = 40 + 20*rand; % 목표 전압 45~55V 랜덤
        V_init = V_ref_val + noise_v; % 현재 전압은 목표 근처에서 시작
        
        Pcpl_val = 500;    % 500W 부하 켜짐
        
        % *팁: 부하가 켜져있으면 인덕터에도 이미 전류가 흐르고 있어야 물리적으로 맞음
        % 평균 전류 I_L = I_Load = P / V
        I_init = Pcpl_val / V_init; 
    end

    % --- Simulink 모델 변수 덮어쓰기 ---
    % Simulink 모델 내의 변수 이름과 일치해야 합니다.
    in = in.setVariable('V_init_Model', V_init);
    in = in.setVariable('I_init_Model', I_init);
    in = in.setVariable('Pcpl_Value', Pcpl_val);
    in = in.setVariable('V_ref', V_ref_val);
    
    disp(['Scenario: ' num2str(scenario_type) ...
          ', Pcpl: ' num2str(Pcpl_val) ...
          ', V_init: ' num2str(V_init)]);
end