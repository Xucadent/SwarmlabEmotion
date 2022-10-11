function map = create_buildings(map)
% CREATE_BUILDINGS - Create the city buildings in grid. The heights are
% random.
%
% Inputs:
%   map: structure of map parameters
%  
% Outputs:
%   map: structure of map parameters
%
    
%     map.buildings_heights = map.max_height*rand(map.nb_blocks*map.nb_blocks,1)+map.max_height;
%     map.buildings_heights = map.max_height
 %   for i=1:10
  %      map.buildings_heights(i)=map.max_height;
  %  end
 
    buildings_north = [30, 70];
    map.buildings_north = repmat(buildings_north', 1, 1);
    map.buildings_east  = repmat(buildings_north, 1, 1);
    map.buildings_east  = map.buildings_east(:);

end