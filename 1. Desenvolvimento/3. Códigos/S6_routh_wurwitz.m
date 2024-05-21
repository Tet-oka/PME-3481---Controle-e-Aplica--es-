%firstly it is required to get first two row of routh matrix
%e=input('enter the coefficients of characteristic equation: ');
%disp('-------------------------------------------------------------------------')
e = str2sym('s^6 + 29.2103*s^5 + 6413.61*s^4 + 4538.93*s^3 + 472119*s^2 - 241.186*s - 25097.3')
syms s
e = fliplr(coeffs(e,s,'all'));
l=length(e);
m=mod(l,2);
if m==0
    a=sym(rand(1,(l/2)));
    b=sym(rand(1,(l/2)));
    for i=1:(l/2)
        a(i)=e((2*i)-1);
        b(i)=e(2*i);
    end
else
    e1=[e 0];
    a=sym(rand(1,((l+1)/2)));
    b=sym([rand(1,((l-1)/2)),0]);
    for i=1:((l+1)/2)
        a(i)=e1((2*i)-1);
        b(i)=e1(2*i);
    end
end
%%now we genrate the remaining rows of routh matrix
l1=length(a);
c=sym(zeros(l,l1));
c(1,:)=a;
c(2,:)=b;
for m=3:l
    for n=1:l1-1
        c(m,n)=-(1/c(m-1,1))*det([c((m-2),1) c((m-2),(n+1));c((m-1),1) c((m-1),(n+1))]);
    end
end
disp('the routh matrix:')
disp(c)
%%now we check the stablity of system
% if c(:,1)>0
%     disp('System is Stable')
% else
%     disp('System is Unstable');
% end
syms k real
ksol = solve(c(:,1)>0,k,'ReturnConditions',true)