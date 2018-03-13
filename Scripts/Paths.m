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