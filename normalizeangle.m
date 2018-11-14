function x = normalizeangle(x)

x(x > pi) = x(x>pi) - 2*pi;
x(x < -pi) = x(x<-pi) + 2*pi;

x(x > pi) = x(x>pi) - 2*pi;
x(x < -pi) = x(x<-pi) + 2*pi;