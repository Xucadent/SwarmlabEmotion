function [map,p_swarm] = move_obstacles(map, p_swarm, p_sim)

edgeR = 80;
edgeL = 20;

for obs = 1 : p_swarm.n_cyl
    if(map.buildings_east(obs) > edgeR && p_swarm.cylinder_vel(2, obs) > 0)
        p_swarm.cylinder_vel(2, obs)  = -p_swarm.cylinder_vel(2, obs);
    elseif(map.buildings_east(obs) < edgeL && p_swarm.cylinder_vel(2, obs) < 0)
        p_swarm.cylinder_vel(2, obs)  = -p_swarm.cylinder_vel(2, obs);
    end 
end
    
    map.buildings_east(1) = map.buildings_east(1) + p_swarm.cylinder_vel(2, 1)*p_sim.dt;
    p_swarm.cylinders(2,1) = p_swarm.cylinders(2,1) + p_swarm.cylinder_vel(2, 1)*p_sim.dt;
    
    map.buildings_east(2) = map.buildings_east(2) + p_swarm.cylinder_vel(2, 2)*p_sim.dt;
    p_swarm.cylinders(2,2) = p_swarm.cylinders(2,2) + p_swarm.cylinder_vel(2, 2)*p_sim.dt;

end

