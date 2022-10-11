function [fig_handle, cylinders]= draw_cylinders(fig_handle, map)
% DRAW_CYLINDERS - plot cylindric obstacles
  
  % Draw buildings    
  NB_EDGES  = 50;
  gray_shade = 0.8;
  
  [X,Y,Z] = cylinder(map.building_width/2,NB_EDGES);

  Z = 1 * Z;
  cylinders = [];
  nb_buildings = length(map.buildings_north);
  for i = 1:nb_buildings
      Xtrasl = X + map.buildings_north(i);
      Ytrasl = Y + map.buildings_east(i);
      C = repmat(gray_shade*ones(size(Xtrasl)),1,1,3);
      cylinder_handle = surf(Ytrasl, Xtrasl, Z, C,'LineWidth',0.5);
      cylinders = [cylinders cylinder_handle];
      hold on;
  end
  
  map_width = map.width;
  axes_lim = [-map_width/5 + map.bl_corner_east+50, ... % x_min
      map_width + map_width/5 + map.bl_corner_east-50, ... % x_max
      -map_width/4 + map.bl_corner_north+50, ... % y_min
      map_width + map_width/4 + map.bl_corner_north-50, ... % y_max
      0, ... % z_min
      1.2*1]; % z_max
  axis(axes_lim);
  axis square;
  view(0,90);

end