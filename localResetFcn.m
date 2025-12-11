function in = localResetFcn(in)
    % 랜덤 초기 상태 생성
    x0 = 40 + rand * (60 - 40);

    % 모델에 전달 (Simulink variables)
    in = setVariable(in,'x0',x0);
end
