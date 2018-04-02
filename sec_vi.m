function [VI] = sec_vi(vi_h,U,alp)
vi(1) = vi_h^2/sqrt((U*cos(alp))^2+(U*sin(alp)+vi_h)^2);
vi(2) = vi_h^2/sqrt((U*cos(alp))^2+(U*sin(alp)+vi_h*.99)^2);
e(1) = 0;
e(2) = abs(vi(2)-vi(1));
i=3;

while e(i-1)>=.00000001
    vi(i) = vi(i-1)-e(i-1)*(vi(i-1)-vi(i-2))/(e(i-1)-e(i-2));
    vi(i) = vi_h^2/sqrt((U*cos(alp))^2+(U*sin(alp)+vi(i))^2);
    e(i) = abs(vi(i)-vi(i-1));
    i=i+1;
end
VI = vi(length(vi));
end