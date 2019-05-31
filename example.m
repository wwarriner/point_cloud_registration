%% set seed
rng( 314159 );

%% read component
[ fv.faces, fv.vertices ] = ...
    CONVERT_meshformat( READ_stl( which( 'bearing_block.stl' ) ) );
triangle_areas = compute_triangle_areas( fv );

%% construct point cloud
v = fv.vertices;
vpc = pointCloud( v );
%pcshow( vpc );

%% sample original fv for pseudo-canonical representation
%  get face areas
%  get total face area
%  probability of sampling face is FA/TFA * sample_count
%  floor( prob ) is number of guaranteed samples
%  frac( prob ) is probability of one more sample
%  randomly determine how many of fracs are yes
%  get total number of samples per face
%  for each face
%   compute n points using:
%   r1 = rand(0,1);
%   r2 = rand(0,1);
%   s1 = sqrt( r1 );
%   d = ( 1 - s1 ) * a + s1 * ( 1 - r2 ) * b + s1 * r2 * c;
% see: https://chrischoy.github.io/research/barycentric-coordinate-for-mesh-sampling/

desired_count = 1e6;
points = sample_fv( ...
    fv, ...
    triangle_areas, ...
    desired_count ...
    );
pp = pointCloud( points );
pcshow( pcdownsample( pp, 'random', 0.01 ) );

%% sample again for moving representation
desired_count = 1e4;
points = sample_fv( ...
    fv, ...
    triangle_areas, ...
    desired_count ...
    );

%% add uniform scaling
scale_ratio = 1.02;
scaled_points = scale_ratio .* points;
%sp = pointCloud( scaled_points );
points = scaled_points;

%% add low-frequency distortion
amplitude = 5; % stl unit
wavelength = 250; % stl unit
factor = 2 .* pi ./ wavelength;
xfun = @(p)p(:,1);
yfun = @(p)p(:,2);
zfun = @(p)( p(:,3) + amplitude .* cos( factor .* p(:,1) ) .^ 2 );
distorted_points = [ ...
    xfun( points ), ...
    yfun( points ), ...
    zfun( points ) ...
    ];
dp = pointCloud( distorted_points );
%pcshow( dp );
points = distorted_points;

%% add jitter
jitter_aspect_ratio = [ 1 1 1 ];
jitter_scale = 1; % stl unit
jitter_amounts = jitter_scale .* jitter_aspect_ratio;
jittered_points = points + ...
    jitter_amounts .* randn( size( points ) );
jp = pointCloud( jittered_points );
%pcshow( jp );
points = jittered_points;

%% add rotation
r = Rotator( [ pi/4, -pi/4 ] );
rotated_points = r.rotate( points );
rp = pointCloud( rotated_points );
%pcshow( rp );
points = rotated_points;

%% register
moving = rp;
fixed = pp;

moving_ds = pcdownsample( moving, 'random', 0.01 );
fixed_ds = pcdownsample( fixed, 'random', 0.01 );
[ tform, moving_rigid, rmse ] = pcregistercpd( ...
    moving_ds, fixed_ds, ...
    'transform', 'rigid', ...
    'maxiterations', 50, ...
    'verbose', true ...
    );
moving_result = pctransform( moving, tform );
% [ tform, reg, rmse ] = pcregistercpd( ...
%     moving_rigid, fixed_ds, ...
%     'transform', 'nonrigid', ...
%     'maxiterations', 50, ...
%     'verbose', true ...
%     );
% reg = pctransform( reg, tform );

%% show transformed set
fh = figure();
fh.Color = 'w';
axh = axes( fh );
hold( axh, 'on' );
ph = patch( axh, fv );
ph.EdgeColor = 'none';
ph.FaceAlpha = 0.5;
ph.FaceColor = [ 0.9 0.9 0.9 ];
p = points;
sh = scatter3( p( :, 1 ), p( :, 2 ), p( :, 3 ) );
sh.Marker = '.';
sh.MarkerFaceColor = 'k';
sh.SizeData = 20;
sh.MarkerEdgeColor = [ 0.0 0.6 0.5 ];
view( 3 );
view( [ 225 45 ] );
light( 'Position', [ -1 0 0 ] );
light( 'Position', [ 1 0 0 ] );
axis( axh, 'equal' );
axh.XLim = [ -250 250 ];
axh.YLim = [ -250 250 ];
axh.ZLim = [ -250 250 ];

%% show registration result
fh = figure();
fh.Color = 'w';
axh = axes( fh );
hold( axh, 'on' );
ph = patch( axh, fv );
ph.EdgeColor = 'none';
ph.FaceAlpha = 0.5;
ph.FaceColor = [ 0.9 0.9 0.9 ];

p = moving_result.Location;
sh = scatter3( p( :, 1 ), p( :, 2 ), p( :, 3 ) );
sh.Marker = '.';
sh.MarkerFaceColor = 'k';
sh.SizeData = 20;
sh.MarkerEdgeColor = [ 0.9 0.6 0.0 ];
%tri = delaunay( p );
%ph = tetramesh( tri, p );
%tri.Alpha = 50;
%ph = plot( tri );
%ph.EdgeColor = 'none';
%ph.FaceAlpha = 0.5;
%ph.FaceColor = [ 0.9 0.1 0.1 ];
% ph = scatter3( ...
%     axh, ...
%     moving_rigid.Location( :, 1 ), ...
%     moving_rigid.Location( :, 2 ), ...
%     moving_rigid.Location( :, 3 ), ...
%     'k.' ...
%     );
% ph.SizeData = 20;
% ph.MarkerFaceAlpha = 0.5;
view( 3 );
view( [ 225 45 ] );
light( 'Position', [ -1 0 0 ] );
light( 'Position', [ 1 0 0 ] );
axis( axh, 'equal' );
%pcshowpair( moving_rigid, fixed_ds );
%pcshowpair( reg, fixed_ds );
axh.XLim = [ -250 250 ];
axh.YLim = [ -250 250 ];
axh.ZLim = [ -250 250 ];