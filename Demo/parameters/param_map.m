%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

map.width = 100; % the map is of size (width)x(width)

if exist('ACTIVE_ENVIRONMENT', 'var')
    map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT; % for those functions calling map
end
    
if ~exist('ACTIVE_ENVIRONMENT', 'var') || ACTIVE_ENVIRONMENT == false     
    return 
end

if exist('ACTIVE_ENVIRONMENT', 'var') && ACTIVE_ENVIRONMENT == true
    map.bl_corner_north = 0;
    map.bl_corner_east = 0;

    map.nb_blocks = 2; % the number of blocks per row
    map.street_width_perc = 0.8; % percentage of block that is empty

    map.building_width = map.width/map.nb_blocks*(1-map.street_width_perc);
    map.street_width = map.width/map.nb_blocks*map.street_width_perc;

    map.building_shape = 'cylinder';
    %map.building_shape = 'parallelepiped';

    % Create buildings parameters
    %map = create_shifted_buildings(map);
     map = create_buildings(map);
end

