
function [ robot ] = ScaraInit(  )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 4
% Solutions by Craig McDonald
% 
%
%    DESCRIPTION - Initialize a structure "robot" to contain important
%    robot information that will be passed into various simulation
%    functions.
%
%    ADDITIONAL CODE NEEDED:
%
%    Provide values for the missing system parameters.
%
%    Provide the transform describing the end-effector frame relative to
%    the last frame determined by D-H.
%
%    Provide the limits of the workspace.
%
% 

robot.m_1 = 1; % [kg]
robot.m_2 = 1; % [kg]
robot.m_3 = 1;
robot.m_r1 = 0; % [kg]
robot.m_r2 = 0; % [kg]
robot.m_r3 = 0;
robot.l_1 = 1; % [m]
robot.l_2 = 1; % [m]
robot.l_3 = 1;
robot.g = 9.81; % [m/s^2]
robot.tool = [];
robot.workspace = [-3, 3, -3, 3, -3, 3]; % only used to determine size of figure window
robot.colors = {[.25,.10,.111],[.23,.44,.10],[.200,.10,.40]};

end

