%% 
% 1) Simulink 환경 객체 만들기
mdl      = 'fSystemRLIM';
agentBlk = [mdl '/RL Agent'];

nx = 3;  % 상태 차원
nu = 1;  % action 차원

obsInfo = rlNumericSpec([nx 1]);
obsInfo.Name = 'state';

actInfo = rlNumericSpec([nu 1], 'LowerLimit', -1, 'UpperLimit', 1);
actInfo.Name = 'action';

env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @localResetFcn;
%% Critic 네트워크 정의
hSize = 32;

statePath = [
    featureInputLayer(nx, 'Normalization','none', 'Name','state')
    fullyConnectedLayer(hSize, 'Name','c_fc1')
    reluLayer('Name','c_relu1')
    fullyConnectedLayer(hSize, 'Name','c_fc2')
    reluLayer('Name','c_relu2')
    ];

actionPath = [
    featureInputLayer(nu, 'Normalization','none', 'Name','action')
    fullyConnectedLayer(hSize, 'Name','c_fc3')
    reluLayer('Name','c_relu3')
    ];

commonPath = [
    additionLayer(2, 'Name','add')
    reluLayer('Name','c_relu4')
    fullyConnectedLayer(hSize, 'Name','c_fc4')
    reluLayer('Name','c_relu5')
    fullyConnectedLayer(1, 'Name','Qvalue')
    ];

criticLG = layerGraph(statePath);
criticLG = addLayers(criticLG, actionPath);
criticLG = addLayers(criticLG, commonPath);

criticLG = connectLayers(criticLG,'c_relu2','add/in1');
criticLG = connectLayers(criticLG,'c_relu3','add/in2');

criticOpts = rlRepresentationOptions( ...
    'LearnRate',5e-4, ...
    'GradientThreshold',1);

critic = rlQValueRepresentation(criticLG, obsInfo, actInfo, ...
    'Observation', {'state'}, ...
    'Action', {'action'}, ...
    criticOpts);

%% Actor 네트워크 정의
actorLayers = [
    featureInputLayer(nx, 'Normalization','none','Name','state')
    fullyConnectedLayer(hSize, 'Name','a_fc1')
    reluLayer('Name','a_relu1')
    fullyConnectedLayer(hSize, 'Name','a_fc2')
    reluLayer('Name','a_relu2')
    fullyConnectedLayer(nu, 'Name','a_fc3')
    tanhLayer('Name','tanh')   % 출력 범위 [-1,1]
    ];

actorLG = layerGraph(actorLayers);

actorOpts = rlRepresentationOptions( ...
    'LearnRate',5e-4, ...
    'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorLG, obsInfo, actInfo, ...
    'Observation', {'state'}, ...
    'Action', {'tanh'}, ...
    actorOpts);

%% Agent 옵션
Ts = 200e-6;   % 에이전트 샘플링 시간 (Simulink RL Agent 블록과 동일하게)

agentOpts = rlDDPGAgentOptions;
agentOpts.SampleTime     = Ts;
agentOpts.DiscountFactor = 0.99;
agentOpts.TargetSmoothFactor = 20e-6;

% 버전에 맞게 필드명 확인 필요
agentOpts.NoiseOptions.Variance          = 0.3;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-4;
agentOpts.NoiseOptions.VarianceMin       = 0.05;   % 또는 MinimumVariance

agent = rlDDPGAgent(actor, critic, agentOpts);




% 1) training options 설정
trainOpts = rlTrainingOptions( ...
    MaxEpisodes        = 2000, ...
    MaxStepsPerEpisode = 500, ...
    StopTrainingCriteria = "EpisodeCount", ...
    StopTrainingValue    = 500, ...   % 그냥 500 에피소드 꽉 채우기 예시
    SaveAgentCriteria    = "EpisodeCount", ...  % 에피소드 수 기준으로 저장
    SaveAgentValue       = 1, ...               % 1 에피소드마다 저장 → Agent1, Agent2, ...
    SaveAgentDirectory   = "savedAgents", ...
    Plots                = "training-progress", ...
    Verbose              = false);
% 1개 출력만!
trainingStats = train(agent, env, trainOpts);

% 최고 reward 에피소드 index
[bestReward, bestEp] = max(trainingStats.EpisodeReward);

% 그 에피소드에 저장된 agent 불러오기
bestFile = fullfile(trainOpts.SaveAgentDirectory, ...
                    sprintf("Agent%d.mat", bestEp));


%% load file
tmp = load("savedAgents\Agent47.mat");
%tmp = load(bestFile);
bestAgent = tmp.saved_agent;

