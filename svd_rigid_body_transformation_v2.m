% Proof of concept for solving motion estimation using SVD
clc;
clear all;
% close all;
tic;
rng(42) % set random seed

big_S = {};
S11 = [];
S22 = [];

% Attempt to iterate through different noise severity and observe S matrix
% changing
for tmp_itr = 1:20

    %% Make sets of points
    % A rotation to eventually solve for
    theta = -pi/8;
    R_offset = [cos(theta) -sin(theta); sin(theta) cos(theta)];

    % P1 is a pointcloud with random coordinates
    N = 5;
    % P1 = 0.25*randn(2,N);

    % Create a nice P1 around the origin, and a P2 that is a rotated version
    % P1 = [-1 -1; -1 1; 1 1; 1 -1]'; % square
    % P1 = [-0.2 -0.2; -0.5 0.5; 1 1; 0.5 -0.5; 0.8 0.8]'; % arrow head
    P1 = [2 0; 1 0; 0 0; 0 0.5; 0 1]';
    P2 = R_offset * P1 + [-2 -3.5]';

    % Wobble the points
    if(tmp_itr>10)
        for i = 1:N
            P2(:,i) = P2(:,i) - tmp_itr * 0.005*randn;% 0.05*randn;
        end
    end

    % Show points
    figure(1);
    clf;
    hold on;
    grid on;
    axis equal;
    scatter(P1(1,:),P1(2,:),100,'ro');
    scatter(P2(1,:),P2(2,:),100,'bo');
    for i = 1:N
        plot([P1(1,i) P2(1,i)], [P1(2,i) P2(2,i)],'k--');
    end
    title('Two sets of N points to solve for rigid body motion');

    %% Prepare cross-dispersion matrix C
    % Find mean x,y of the set of points
    P1_mean = mean(P1,2);
    P2_mean = mean(P2,2);

    scatter(P1_mean(1,:),P1_mean(2,:),100,'r.');
    scatter(P2_mean(1,:),P2_mean(2,:),100,'b.');

    % Transform the points over to the origin
    P1_origin = P1 - P1_mean;
    P2_origin = P2 - P2_mean;

    scatter(P1_origin(1,:),P1_origin(2,:),25,'rx');
    scatter(P2_origin(1,:),P2_origin(2,:),25,'bx');

    W = eye(N); % weights

    C = P1_origin * W * P2_origin';

    %% Do SVD and find rotation
    [U,S,V] = svd(C);
    M = eye(2);
    M(2,2) = det(U*V');
    R_without_M = U*V';
    R_final = U*M*V';

    R_final = R_final'; % the reflection comes out backwards unless we used V*U'
    % there is an inconsistency between Challis 1995 and Arun 1987 I think

    theta_R = atan2(R_final(2,1),R_final(1,1));
    theta_R_no_M = atan2(R_without_M(2,1),R_without_M(1,1));

    %% Transform vector v
    v = P2_mean - R_final * P1_mean;

    % Move P1 points back onto P2, showing v is correct
    P2_restored = R_final * P1 + v;
    scatter(P2_restored(1,:),P2_restored(2,:),100,'g+');
    
    big_S = [big_S; {S}];
    S11 = [S11;S(1,1)];
    S22 = [S22;S(2,2)];


end

%%
figure(2);
clf;
hold on;
grid on;
% axis equal;
% for i = 1:size(big_S)
%     plot(i,big_S{i,1}(1,1))
%     scatter(big_S{i,1}(1,1),big_S{i,1}(2,2))
% end

plot(S11);
plot(S22);




