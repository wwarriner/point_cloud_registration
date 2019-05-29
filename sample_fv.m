function points = sample_fv( ...
    fv, ...
    triangle_areas, ...
    desired_sample_count ...
    )

total_area = sum( triangle_areas, 'all' );
probabilities = desired_sample_count .* triangle_areas ./ total_area;
random_values = double( rand( size( probabilities ) ) );
thresholds = probabilities - fix( probabilities );
samples_per_face = floor( probabilities ) + ( random_values > thresholds );

faces = fv.faces;
vertices = fv.vertices;

sample_count = sum( samples_per_face, 'all' );
points = nan( sample_count, 3 );
face_count = numel( triangle_areas );
j = 1;
for i = 1 : face_count
    
    sc = samples_per_face( i );
    if sc <= 0
        continue;
    end
    r1 = rand( sc, 1 );
    r2 = rand( sc, 1 );
    s1 = sqrt( r1 );
    points( j : j + sc - 1, : ) = ...
        ( 1 - s1 ) .* vertices( faces( i, 1 ), : ) + ...
        s1 .* ( 1 - r2 ) .* vertices( faces( i, 2 ), : ) + ...
        s1 .* r2 .* vertices( faces( i, 3 ), : );
    j = j + sc;
    
end

% pp = pointCloud( sampled_points );
% pcshow( pp );

actual_count = size( points, 1 );
overshoot = actual_count - desired_sample_count;
%assert( overshoot >= 0 );
if overshoot > 0
    remove_indices = randsample( actual_count, overshoot );
    points( remove_indices, : ) = [];
end

end

