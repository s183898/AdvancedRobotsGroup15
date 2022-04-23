%% Wavefront planner
clc
clear all
close all

map = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        NaN, NaN, NaN, NaN, NaN, NaN, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, NaN, NaN, NaN, NaN, NaN, NaN, NaN;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

q = [];
start = [2,2];
goal = [9,9];

new_map = makeWave(map,start,goal,q)

route = routeFinder(new_map,start,goal);
    
mapSolved = map;

for i = 1:length(route(:,1))
    move = route(i,:);
    mapSolved(move(1),move(2))=1;
    if sum(move==start) == 2
        mapSolved(move(1),move(2))=2;
    end
end







