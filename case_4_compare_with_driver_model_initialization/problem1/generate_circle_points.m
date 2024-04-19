function [x_values, y_values]= generate_circle_points(rx, ry, n, num_points)
    t_values = linspace(pi, 0, num_points);
    x_values = rx*sign(cos(t_values)).*abs(cos(t_values)).^(2/n);
    y_values = ry*sign(sin(t_values)).*abs(sin(t_values)).^(2/n);
end 