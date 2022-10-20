classdef Emotion

    properties
        emotion_L
        emotion_H
        time_steps
        transfer_prop
        fear_dist
    end

    properties
        status
        frust
        fear
    end

    methods
        function emotion = Emotion(p_emotion)
            emotion.status = EmotionEnum.Neutral;
            emotion.frust = 0;
            emotion.fear = 0;
            
            emotion.emotion_L = p_emotion.emotion_L;
            emotion.emotion_H = p_emotion.emotion_H;
            emotion.time_steps = p_emotion.time_steps;
            emotion.transfer_prop = p_emotion.transfer_prop;
            emotion.fear_dist = p_emotion.fear_dist;
        end

        function emotion = calc_frust(self, vel_history)
            time_len = size(vel_history, 1);
            if time_len <= self.time_steps
                time_0 = 1;
            else
                time_0 = time_len - self.time_steps;
            end
            velsum_scaler = 0;
            velsum_vector = zeros(1, 2);

            for i = time_0 : time_len
                velsum_vector = velsum_vector + vel_history(i, :);
                velsum_scaler = velsum_scaler + norm(vel_history(i, :));
            end

            emotion = self;
            if velsum_scaler > 0
                emotion.frust = 1 - norm(velsum_vector) / velsum_scaler;
            else
                emotion.frust = 1;
            end
            
        end

        function emotion = calc_fear(self, neigfear_num, neig_num, dist_obs)
            emotion = self;
            if neig_num ~= 0
                if dist_obs < self.fear_dist
                    % emotion.fear = self.transfer_prop * (neigfear_num / neig_num) + (1 - self.transfer_prop) * (1 - dist_obs / self.fear_dist);
                    % emotion.fear = self.transfer_prop * (neigfear_num / neig_num) + (1 - self.transfer_prop) * (-1 + 2 / (dist_obs / self.fear_dist + 1));
                    % emotion.fear = self.transfer_prop * (neigfear_num / neig_num) + (1 - self.transfer_prop) * (1 - dist_obs / self.fear_dist).^2;
                    emotion.fear = self.transfer_prop * (neigfear_num / neig_num) + (1 - self.transfer_prop) * (1 - dist_obs / self.fear_dist).^2;
                else
                    emotion.fear = self.transfer_prop * (neigfear_num / neig_num);
                end
            else
                emotion.fear = (1 - self.transfer_prop) * (1 - dist_obs / self.fear_dist).^2;
            end

            if emotion.fear > 1
                emotion.fear = 1;
            elseif emotion.fear < 0
                emotion.fear = 0;
            end
        end

        function emotion = update_emotion(self)
            emotion = self;
            if self.frust < self.emotion_L && self.fear < self.emotion_L
                emotion.status = EmotionEnum.Neutral;
            elseif self.frust > self.emotion_H && self.fear < self.emotion_L
                emotion.status = EmotionEnum.Frustration;
            elseif self.frust < self.emotion_L && self.fear > self.emotion_H
                emotion.status = EmotionEnum.Fear;
            end
        end
    end
end
