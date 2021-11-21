function T = FKShell(theta)

l1 = 425;
l2 = 392;
w1 = 133;
w2 = 99.6;
h1 = -240;
h2 = 99.7;

b_1 = transpose([0, 1, 0, w1 + w2, 0, l1 + l2]);
b_2 = transpose([0, 0, 1, h2, -(l1 + l2), 0]);
b_3 = transpose([0, 0, 1, h2, -l2, 0]);
b_4 = transpose([0, 0, 1, h2, 0, 0]);
b_5 = transpose([0, -1, 0, -w2, 0, 0]);
b_6 = transpose([0, 0, 1, 0, 0, 0]);
body_screws = [b_1 b_2 b_3 b_4 b_5 b_6];

m = [
        1, 0, 0, (-(l1 + l2));
        0, 0, (-1), (-(w1 + w2));
        0, 1, 0, (h1 - h2);
        0, 0, 0, 1;
];

T = m * get_exp(1, body_screws, theta) * get_exp(2, body_screws, theta) * get_exp(3, body_screws, theta) * get_exp(4, body_screws, theta) * get_exp(5, body_screws, theta) * get_exp(6, body_screws, theta);
end
function lt = lt_cross(w)
    lt = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
end
function result = bracket_s(s)
    top_left = lt_cross(s(1:3));
    top_right = s(4:6);
    bottom = [0 0 0 0];
    result = [top_left top_right; bottom];
end 
function result = get_exp(i, body_screws, theta)
    result = expm(bracket_s(body_screws(1:6, i)) * theta(i));
end

