p_emotion.init = 0;

if ~isfield(p_emotion, 'emotion_L')
    p_emotion.emotion_L = 0.1;
end

if ~isfield(p_emotion, 'emotion_H')
    p_emotion.emotion_H = 0.2;
end

if ~isfield(p_emotion, 'time_steps')
    p_emotion.time_steps = 200;
end

if ~isfield(p_emotion, 'transfer_prop')
    p_emotion.transfer_prop = 0.2;  % 情感传递影响因子
    
end

if ~isfield(p_emotion, 'fear_dist')
    p_emotion.fear_dist = 30;      % 恐惧情感激发距离
end
