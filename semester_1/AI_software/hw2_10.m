%task 10
N = randi(90,8);
n = randi(7);
function[new_matrix] = topright(N,n)
    new_matrix = [N(1:n,end-n+1:end)];
end