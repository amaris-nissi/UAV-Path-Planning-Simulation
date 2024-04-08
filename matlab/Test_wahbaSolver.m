clear;
clc;

x=0;
y=0;

for i = 1 : 1
     e = [pi/4;2;1];
%     e = [pi/3;1;3];
%     e = [pi;1;1];
%     e = [pi/3;pi/4;1];

    RBI_euler = euler2dcm(e);
    N = 10;
    aVec = rand(N,1);
    vI_mat = rand(N,3);
    vB_mat = (RBI_euler * vI_mat')' ;

    RBI_Wahba = wahbaSolver(aVec,vI_mat,vB_mat);

    E = RBI_euler * transpose(RBI_Wahba);
    disp(E)

end