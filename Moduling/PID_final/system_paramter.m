sys=LinearAnalysisToolProject.Results.Data.Value;
A=sys.a;
B=sys.b;
 c=[1 0 0 0 0 0 0 0;0 0 1 0 0 0 0 0;0 0 0 0 1 0 0 0;0 0 0 0 0 0 1 0];
D=sys.d;
q=[25     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0    25     0     0     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     0     0    75     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0    50     0;
     0     0     0     0     0     0     0     0];
 
r=0.01*diag(ones(1,4));
[k s e]=lqr(A,B,q,r);k

