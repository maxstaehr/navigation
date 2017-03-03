A = zeros(5);
for row = 1: 5
    for column = row+1:5
        A(row, column) = 1;
    end
end
disp(A)