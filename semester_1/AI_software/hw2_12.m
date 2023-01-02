%hw2_12

N = randi(90,5);
new_matr = transform(N);

function matr = transform(n)
size_n = size(n);
size_n = size_n(1);
matr = zeros(size_n,size_n);
    for i = 1:size_n
        for j = 1:size_n
            if or((i == j),(i + j == size_n +1))
                matr(i,j) = n(i,j);
            end
        end
    end
end
