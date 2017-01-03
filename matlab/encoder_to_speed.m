function [ w ] = encoder_to_speed(encoder, Ts, Np)
%ENCODER_TO_SPEED 
th = 1;
n=length(encoder);
rising_edge = [0; (encoder(1:n-1)<th).*(encoder(2:n)>th)==1];

w = zeros(n,1);
j = 0; % High freq counter
first_sample = true;
current_speed = 0.0;
for i=1:n
  w(i) = current_speed;
  if (rising_edge(i))
    current_speed = 2*pi/(Np*j*Ts);
    j = 0;
    if first_sample
        current_speed = 0.0;
        first_sample = false;
    end
  end
  j = j + 1;
end


end
