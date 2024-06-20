clear;close all;clc
load("targetLostTime.mat")

[N,edges] = histcounts(targetlosttimes1,20,'Normalization', 'probability');
xbar = edges(1:length(N)) + mean(diff(edges))/2;

figure(1)
bar(xbar,N)
title("Target lost time")
grid on
grid minor
