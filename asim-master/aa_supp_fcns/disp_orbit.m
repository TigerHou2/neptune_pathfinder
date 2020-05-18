function disp_orbit(R,V,mu,dur,res,start_day,color,width)
%DISP_ORBIT Plots trajectory given pos, vel vectors and optional arguments
%Author: Tiger Hou
%   R        : m       , 3-by-1 column vector
%   V        : m/s     , 3-by-1 column vector
%   mu       : m^3/s^2 , gravitational parameter of central body
%   dur      : days    , duration of simulation
%   res      : nd      , resolution of sim (for hyperbolic cases)
%   start_day: days    , number of days after R,V point to start plotting
%   color    : string  , color of orbit plot
%   width    : nd      , width of orbit plot

if ~exist('res','var')
    res = 50;
end
if ~exist('start_day','var')
    start_day = 0;
end
if ~exist('color','var')
    color = 'red';
end
if ~exist('width','var')
    width = 1;
end

pos = Get_Orb_Points(R(:,end),V(:,end),mu,res,dur,start_day);
plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',width,'Color',color);

view(cross(R(:,end),V(:,end)))
pbaspect([1 1 1])
axis equal
grid on

end

