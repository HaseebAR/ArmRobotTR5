system=LinearAnalysisToolProject.Results.Data.Value;
a=system.a;
b=system.b;
c=[1 0 0 0 0 0 0 0;0 0 1 0 0 0 0 0;0 0 0 0 1 0 0 0;0 0 0 0 0 0 1 0];
d=system.d;

q=[50     0     0     0     0     0     0     0;
     0     1     0     0     0     0     0     0;
     0     0    1000    0     0     0     0     0;
     0     0     0     10   0     0     0     0;
     0     0     0     0   100    0     0     0;
     0     0     0     0     0    1     0     0;
     0     0     0     0     0     0   100     0;
     0     0     0     0     0     0     0     1];
r=0.1*[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
[k,s,e]=lqr(a,b,q,r);
M=inv(c*inv(a+b*k)*b)