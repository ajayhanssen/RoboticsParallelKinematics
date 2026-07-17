function [s1, s2, s3] = inverse_kinematics(x, y, z, rb, rm, l)

    % Thetas
    Thetam1 = 0;
    Thetam2 = 240;
    Thetam3 = 120;
    
    % mis
    xm1 = x*cosd(Thetam1) + y*sind(Thetam1);
    xm2 = x*cosd(Thetam2) + y*sind(Thetam2);
    xm3 = x*cosd(Thetam3) + y*sind(Thetam3);
    
    ym1 = y*cosd(Thetam1) - x*sind(Thetam1) + rm;
    ym2 = y*cosd(Thetam2) - x*sind(Thetam2) + rm;
    ym3 = y*cosd(Thetam3) - x*sind(Thetam3) + rm;
    
    zm1 = z;
    zm2 = z;
    zm3 = z;
    
    % a, b, c
    a1 = 1;
    a2 = 1;
    a3 = 1;
    
    b1 = -2*zm1*sind(45) - 2*rb*cosd(45) + 2*ym1*cosd(45);
    b2 = -2*zm2*sind(45) - 2*rb*cosd(45) + 2*ym2*cosd(45);
    b3 = -2*zm3*sind(45) - 2*rb*cosd(45) + 2*ym3*cosd(45);
    
    c1 = xm1^2 + ym1^2 + zm1^2 + rb^2 - 2*ym1*rb - l^2;
    c2 = xm2^2 + ym2^2 + zm2^2 + rb^2 - 2*ym2*rb - l^2;
    c3 = xm3^2 + ym3^2 + zm3^2 + rb^2 - 2*ym3*rb - l^2;
    
    % quad eq
    Delta1 = b1^2 - 4*a1*c1;
    Delta2 = b2^2 - 4*a2*c2;
    Delta3 = b3^2 - 4*a3*c3;
    
    s1 = (-b1-sqrt(Delta1))/2;
    s2 = (-b2-sqrt(Delta2))/2;
    s3 = (-b3-sqrt(Delta3))/2;

end