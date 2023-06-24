lc=0.11;
l=0.2;
cs=1;
if l>lc
    nf=6*cs*lc^(-1.6);
end
if l>=lc
    fl=1-lc^1.71*l^(-1.71);
else
    fl=0;
end
xita=log10(l);

if xita<=-1.4
    e=1;
elseif xita>-1.4 && xita<0
    e=1-0.3571*(xita+1.4);
else
    e=0.5;
end

if xita<=-0.5
    mu1=0.45;
elseif xita>-0.5 && xita<0
    mu1=-0.45-0.9*(xita+0.5);
else
    mu1=-0.9;
end
mu2=-0.9;
sigma1=0.55;

if xita<=-1.0
    sigma2=0.28;
elseif xita>-1.0 && xita<0.1
    sigma2=0.28-0.1636*(xita+1);
else
    sigma2=0.1;
end
gama = -10:0.01:10;
p1=normpdf(gama, mu1, sigma1);
p2=normpdf(gama, mu2, sigma2);
p=e*p1+(1-e)*p2;
plot(gama,p);
grid on;