function deviation = pathDeviation(XYZ, wayp)

    XYZ = XYZ(:)';
    N = size(XYZ, 1);
    
    num_segments = size(wayp, 1) - 1;
    dist_to_segments = zeros(N, num_segments);
    
    % loop every line segment of waypoints
    for i = 1:num_segments
        A = wayp(i, :);
        B = wayp(i+1, :);
    
        AB = B - A;
        AB_sq = max(dot(AB, AB), eps);
    
        AP = XYZ - A;                
    
        dot_AP_AB = AP(:,1)*AB(1) + AP(:,2)*AB(2) + AP(:,3)*AB(3);
        r = dot_AP_AB / AB_sq;
    
        % t < 0: closest point is A
        % t > 1: closest point is B
        % 0 <= t <= 1: closest point lies on line segment AB
        r = max(0, min(1, r)); 
    
        % find closest point on segment each trajectory point
        closest_pts = A + r .* AB;
    
        % euclidean distance
        diffs = XYZ - closest_pts;
        dist_to_segments(:, i) = sqrt(sum(diffs.^2, 2));
    end
    
    % deviation is distance to closest segment
    deviation = min(dist_to_segments, [], 2);

end