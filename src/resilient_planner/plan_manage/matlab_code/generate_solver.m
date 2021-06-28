% The code is about the nmpc with external forces

clear
close all
clearvars -global
clc

addpath(genpath(pwd));

setup;


mpc_generator_normal;
mpc_generator_final;