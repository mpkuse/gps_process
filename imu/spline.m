clear all;
close all;

P0 = [ 2 ; 3];
P1 = [ 4 ; 1];
P2 = [ 4 ; 2];
P3 = [ 3 ; 6];

P = [ P0 P1 P2 P3 ];

plot( P( 1, : ), P( 2, : ), 'g*' );
axis( [-10 10 -10 10] );
hold on

tic
count=1;
for u=0:0.0001:1
    Q(:,count) = (1-u)^3*P0 + 3*(1-u)^2*u*P1 + 3*(1-u)*u^2*P2 + u^3*P3;
    count = count+1;
end
toc


tic
u = linspace( 0, 1, 10000 )';
U = [ (1-u).^3   3*(1-u).^2.*u   3*(1-u).*u.^2    u.^3 ]';
Q = P*U;
toc


plot( Q( 1, : ), Q( 2,: ), 'r.' );
