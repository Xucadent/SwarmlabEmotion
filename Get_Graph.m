scenario_num = size(Scenario_Result, 1);
repeat_times = size(Scenario_Result, 2);

coll_awa = zeros(9,15);
coll_awo = zeros(9,15);
flight_time = zeros(9,15);
for i = 1:9
    for j = 1:15
        for k = 1:repeat_times
            coll_awa(i,j) = coll_awa(i,j) + Scenario_Result{(i-1)*15+j,k}.stat_collisions(1);
            coll_awo(i,j) = coll_awo(i,j) + Scenario_Result{(i-1)*15+j,k}.stat_collisions(2);
            flight_time(i,j) = flight_time(i,j) + Scenario_Result{(i-1)*15+j,k}.stat_time(2) - Scenario_Result{(i-1)*15+j,k}.stat_time(1);
        end
        coll_awa(i,j) = coll_awa(i,j) / repeat_times;
        coll_awo(i,j) = coll_awo(i,j) / repeat_times;
        flight_time(i,j) = flight_time(i,j) / repeat_times;
    end
end

