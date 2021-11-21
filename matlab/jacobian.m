function j = jacobian(body_screws, thetas)

curr_t = eye(4);
j = [];
sz = size(body_screws);
for i_ = 1:sz(2)
    i = sz(2) - i_ + 1;
    curr_transform = t_adjoint(curr_t);
    screw = body_screws(1:6, i);
    j = [(curr_transform * screw) j];
    curr_t = curr_t * expm(screw, - thetas(i));
end

end