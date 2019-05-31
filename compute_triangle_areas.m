function areas = compute_triangle_areas( fv )

u = ( fv.vertices( fv.faces( :, 3 ), : ) - fv.vertices( fv.faces( :, 1 ), : ) ).';
v = ( fv.vertices( fv.faces( :, 3 ), : ) - fv.vertices( fv.faces( :, 2 ), : ) ).';
sq = vecnorm( u ) .* vecnorm( v );
sin_theta = sqrt( 1 - ( ( dot( u, v ) ./ sq ) .^ 2 ) );
areas = ( ( sq .* sin_theta ) ./ 2 ) .';

end

