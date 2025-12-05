function in = localResetFcn(in)
    % 랜덤 초기 상태 생성
    x0 = 0.5 * randn(2,1);

    % 모델에 전달 (Simulink variables)
    in = setVariable(in,'x0',x0);
end
