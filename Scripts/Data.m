%% Pick&Place low
%-- set positions
Home = rob.homeJointTargetPositions;
PickUp = [45 -125 -100 -135 -135 0];
PickUpApp = [45 -113.8520 -93.5075 -152.6405 -135 0];
Place = [-25 -125 -100 -135 -25 0];
PlaceApp = [-25 -113.8520 -93.5075 -152.6405 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%% Pick&Place high
%-- set positions
Home = rob.homeJointTargetPositions;
PickUp = [45 -110 -80 -170 -135 0];
PickUpApp = [45 -113.2953  -44.7716 -201.9331 -135 0];
Place = [-25 -110 -80 -170 -25 0];
PlaceApp = [-25 -113.2953  -44.7716 -201.9331 -25 0];

%-- create path
Path =[Home;PickUpApp;PickUp;PickUpApp;PlaceApp;Place;PlaceApp;Home];

%% Camera Positions
cam.moveToCameraLocation([0 2 1 90 0 0]);       % north
cam.moveToCameraLocation([3 0 1 0 -90 -90]);    % east
cam.moveToCameraLocation([0 -2 1 -90 0 -180]);  % south
cam.moveToCameraLocation([-2 0 1 90 0 0]);      % west

cam.moveToCameraLocation([-1.5 1.5 1 90 45 0]); % north-west