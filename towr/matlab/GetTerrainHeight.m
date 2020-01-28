function h = GetTerrainHeight(x, type)

h = zeros(size(x));
        
switch type
    case "Flat"
        h = zeros(size(x));
    case "Rough"
        rough_start = 0.5;
        freq = 5.0;
        amp = 0.1;
        n_cycles = 2.0;
        rough_end = n_cycles*2*pi/freq + rough_start;
        slope = 0.2;
        h_end = amp*sin(freq*(rough_end-rough_start))+slope*(rough_end-rough_start);
        if x >= rough_start 
            h = amp*sin(freq*(x-rough_start))+slope*(x-rough_start);
        end
        if x >= rough_end
            h = h_end;
        end
    case "SineLowFreq"
        h_offset = 0.2;
        amp = 0.2;
        freq = 2;
        sine_start = 0.5;
        n_cycles = 2;
        sine_end = n_cycles*2*pi/freq + sine_start;
        if x>=sine_start && x<=sine_end
            h = amp*sin(freq*(x-sine_start))+h_offset;
        else
            h = h_offset;
        end
    case "SineHighFreq"
        amp = 0.06; %0.1;
        freq = 2*pi/0.7;
        sine_start = 0.5;
        h_offset = amp;
        n_cycles = 3;
        sine_end = n_cycles*2*pi/freq + sine_start;
        if x>=sine_start && x<=sine_end
            h = amp*sin(freq*(x-sine_start))+h_offset;
        else
            h = h_offset;
        end
    case "Step"
       step_start = 1.0;
       height = 0.2;
       slope = 2.182178902359924; % 10;
       step_end = step_start + height/slope;
       for i = 1:length(x)
         if x(i) >= step_start 
            h(i) = slope*(x(i)-step_start);
         end
         if x(i) >= step_end
            h(i) = height;
         end
       end
  case "Step45"
       step_start = 1.0;
       step_end = 1.2;
       height = 0.2;
       slope = 1;
       for i = 1:length(x)
           if x(i) >= step_start 
               h(i) = slope*(x(i)-step_start);
           end
           if x(i) >= step_end
               h(i) = height;
           end
       end
    case "Slope"
       slope_start = 0.5; 
       up_length   = 2.0;
       down_length = 2.0;
       plat_length = 1.0;
       slope = 0.3;
       height_center = up_length*sin(slope);
       x_plat_start = slope_start+up_length*cos(slope);
       x_down_start = x_plat_start+plat_length;
       x_flat_start = x_down_start + down_length*cos(slope);
        if (x >= slope_start)
            h = slope*(x-slope_start);
        end
        if (x >= x_plat_start)
            h = height_center;
        end
        if (x >= x_down_start)
            h = height_center - slope*(x-x_down_start);
        end
        if (x >= x_flat_start)
            h = 0.0; 
        end
    case "Block"
        block_start = 0.7;
        len = 3.5;
        height = 0.2;
        eps = 0.03;
        slope = height/eps;
        for i = 1:length(x)
           h(i) = 0;
           if (block_start <= x(i) && x(i) <= block_start+eps)
               h(i) = slope*(x(i)-block_start);
           end
           if (block_start+eps <= x(i) && x(i) <= block_start+len)
               h(i) = height;
           end
        end
    case "Gap"
        gap_start = 0.7; %1.0;
        w = 0.3; %0.3;
        hh = 1.0; %1.5;
        slope = hh/w;
        dx = w/2.0;
        xc = gap_start + dx; 
        gap_end_x = gap_start + w;  
        a = (4*hh)/(w*w);
        b = -(8*hh*xc)/(w*w);
        c = -(hh*(w - 2*xc)*(w + 2*xc))/(w*w);
        for i = 1:length(x)
           if (gap_start <= x(i) && x(i) <= gap_end_x)
               h(i) = a*x(i)^2 + b*x(i) + c;
           else
               h(i) = 0.0;
           end
        end
    otherwise
        h = 0.0;
end


end