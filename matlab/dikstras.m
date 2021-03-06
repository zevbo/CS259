function [points, previous, goal_pos] = dikstras(grid, start)

empty = 0;
occ = 1;
goal = 2;
width = length(grid(1,:));
height = length(grid(:,1));

previous = zeros(height, width);
queue = java.util.LinkedList;

goal_found = false;
goal_pos = [0,0];

to_check = [1,0;0,1;-1,0;0,-1];

queue.add([start(1), start(2), start(1), start(2)])
previous(start(1), start(2)) = compress(width, height, -1, -1);

while(not(goal_found))
    el = queue.remove();
    x = el(1);
    y = el(2);
    prev_x = el(3);
    prev_y = el(4);
    val = get_val(grid, occ, width, height, x, y);
    if not(val == occ)
        previous(x,y) = compress(width, height, prev_x, prev_y);
        if val == goal 
           goal_found = true;
           goal_pos(1) = x;
           goal_pos(2) = y;
        else
            for i = 1:4
                to_check_el = to_check(i,:);
                x_check = x + to_check_el(1); 
                y_check = y + to_check_el(2);
                queue.add([x_check, y_check, x, y]);
            end
        end 
    end
end 

points = [];

pos = [0,0];
len = 0;
pos(1) = goal_pos(2);
pos(2) = goal_pos(1);
while not(pos(1) == start(1) && pos(2) == start(2))
    len = len + 1;
    points(len,:) = pos;
    [x, y] = decompress(width, height, previous(pos(1), pos(2)));
    pos(1) = x;
    pos(2) = y;
end

points = transpose(flip(points));

end

function res = compress(width, height, x, y)
    res = (x - 1) + (y - 1) * width;
end
function [x, y] = decompress(width, height, compressed)
    x = mod(compressed, width) + 1;
    u1 = fix(compressed / width);
    y = u1 + 1;
end

function val = get_val(grid, occ, width, height, x, y)
    if x < 1 || x > width || y < 1 || y > height
        val = occ;
    else  
        val = grid(x, y);
    end
end
