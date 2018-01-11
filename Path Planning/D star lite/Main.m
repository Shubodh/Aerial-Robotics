% MAIN
clear; clc;
BIG=1e6;
start=3365; goal=294; load('DATA60x60.mat');
h=L_heuristic;
grid(grid==0)=BIG;
fn_DstarLite( start,goal,grid,h,NC,BIG ); % obatcles at center - basic